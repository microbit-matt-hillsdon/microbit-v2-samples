#include "MicroBit.h"
#include "Tests.h"

//
// Steps 1 to 5 of the log-over-serial work.
//
// Step 1: prove bytes travel both ways between the host and the nRF52,
// through DAPLink. Send '?' and the micro:bit answers.
//
// Step 2: answer "how much data is there?" with 'L'.
//
// Step 3: answer "give me a chunk" with 'R <index> <count>'.
//
// Step 4: survive a shared, lossy wire. Every reply is a frame: a record
// separator (0x1E), a type character, decimal arguments, and a newline. Data
// chunks carry a CRC so the host can tell a damaged chunk from a good one.
//
// Step 5: follow a growing log. 'S 1' subscribes to end-of-data notices, and
// the device sends 'N <end> <hdr>' when the log grows. Rows are never pushed:
// the notice only says where the data now ends, and the host reads the new
// range with 'R' as usual. A lost or stale notice is harmless because each
// one carries the absolute end, not a delta. 'H' returns the column header
// row, for a host that subscribes part way through.
//
// Send 'M 1' to turn on log mirroring. That puts unframed row text on the
// same wire, which is what a user's own program output looks like, and is the
// only way to see the framing doing its job.
//
// Requests are whole lines, so press Enter after each one. A leading 0x1E on
// a request is accepted but not required, so requests can be typed by hand in
// a terminal.
//
// '?' sends 2KB of filler and then answers, to prime DAPLink's serial buffer
// without the reply being the thing the buffer throws away. See send_padding.
// '? 0' answers without the filler.
//
// The host side is not here: retrying a chunk on a bad CRC, spotting
// <DAPLink:Overflow> in the stream, and noticing the log was cleared are all
// the client's job. The device only has to describe what it sent accurately.
//
// Logging is off until button A is pressed, and button B stops it again. The
// device starts stopped so a host can connect, look at what is already in the
// log, and only then start adding to it. A and B together clear the log, and
// leave logging stopped.
//
// The top left pixel is lit while logging. The centre pixel lights for each
// completed request, so a dead link can be told apart from a dead reply: pixel
// but no text means transmit is broken, no pixel at all means receive is.
//

#define BRIDGE_RS               0x1E
#define BRIDGE_VERSION          1
#define BRIDGE_MAX_REQUEST      32
#define BRIDGE_MAX_CHUNK        64
#define BRIDGE_TX_BUFFER        128
#define BRIDGE_NOTIFY_INTERVAL  250
#define BRIDGE_PRIME_BYTES      2048

// Flags reported in the hello reply.
#define BRIDGE_FLAG_PRESENT     0x01
#define BRIDGE_FLAG_FULL        0x02
#define BRIDGE_FLAG_MIRRORING   0x04
#define BRIDGE_FLAG_SUBSCRIBED  0x08
#define BRIDGE_FLAG_LOGGING     0x10

// Where the "logging is running" indicator sits, kept clear of the centre
// pixel the request indicator uses.
#define BRIDGE_PIXEL_LOGGING_X  0
#define BRIDGE_PIXEL_LOGGING_Y  0

static char     requestBuffer[BRIDGE_MAX_REQUEST];
static uint8_t  chunkBuffer[BRIDGE_MAX_CHUNK];
static char     hexBuffer[2 * BRIDGE_MAX_CHUNK + 1];
static bool     mirroring = false;
static bool     subscribed = false;
static bool     readInProgress = false;
static bool     logging = false;
static uint32_t lastNotifiedEnd = 0;

/**
 * Button A starts logging, button B stops it.
 *
 * Both run on the scheduler rather than in the interrupt that read the pin,
 * because listen() defers them by default, so touching the display here is
 * safe. Writing the flag is a single aligned store, so the logging fiber
 * cannot see a half written value.
 */
static void
onButtonA(Event)
{
    logging = true;
    uBit.display.image.setPixelValue(BRIDGE_PIXEL_LOGGING_X, BRIDGE_PIXEL_LOGGING_Y, 255);
}

static void
onButtonB(Event)
{
    logging = false;
    uBit.display.image.setPixelValue(BRIDGE_PIXEL_LOGGING_X, BRIDGE_PIXEL_LOGGING_Y, 0);
}

/**
 * A and B together: throw the log away and start again.
 *
 * Logging is stopped first, so the fiber cannot append a row between the erase
 * and the user seeing an empty log. Pressing A starts it again.
 *
 * A subscribed host is not told directly. notify_fiber sees the end of the data
 * drop to zero within 250ms and sends the usual 'N' notice, which is already
 * defined as "the data now ends here" rather than "the data grew", so a host
 * that handles a shrinking end correctly needs no new message for this.
 *
 * A read that is already in flight can return bytes from the old log. The host
 * is expected to discard anything it read before a notice that moved backwards,
 * so this is left as it is rather than locking the two against each other.
 */
static void
onButtonAB(Event)
{
    logging = false;

    uBit.log.clear();

    uBit.display.printChar('C');
    uBit.sleep(500);
    uBit.display.clear();
}

/**
 * Write one row per second while logging is on.
 *
 * The flag is tested every second rather than the fiber being started and
 * stopped, so a press of A always takes effect within a second and there is
 * only ever one of these running.
 */
static void
log_row_fiber()
{
    uBit.log.setTimeStamp( TimeStampFormat::Seconds );

    while(1)
    {
        if ( logging && !uBit.log.isFull() )
        {
            uBit.log.beginRow();
            uBit.log.logData( "temperature", ManagedString(uBit.thermometer.getTemperature()) );
            uBit.log.logData( "light", ManagedString(uBit.display.readLightLevel()) );
            uBit.log.endRow();
        }

        uBit.sleep(1000);
    }
}

/**
 * CRC-16/XMODEM: polynomial 0x1021, initial value 0x0000, no input or output
 * reflection, no final xor.
 *
 * Hand written rather than using the SDK's crc32_compute because the host has
 * to produce the identical value, and this is small enough to reimplement
 * without ambiguity.
 */
static uint16_t
crc16(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0;

    while (len--)
    {
        crc ^= (uint16_t)(*data++) << 8;

        for (int i = 0; i < 8; i++)
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
    }

    return crc;
}

/**
 * Send one frame: separator, body, newline.
 *
 * The transmit buffer is raised at startup so a whole frame fits in one call,
 * which keeps it from being split by another fiber writing to the same port.
 */
static void
send_frame(ManagedString body)
{
    uBit.serial.send( ManagedString((char)BRIDGE_RS) + body + "\r\n" );
}

static void
send_error(int code)
{
    send_frame( ManagedString("E ") + ManagedString(code) );
}

/**
 * Send an end-of-data notice.
 *
 * The header counter is always zero. Reporting a real value needs to know when
 * MicroBitLog writes a fresh header row, and both headingCount and
 * headingsChanged are private with no public accessor. See the handoff doc.
 */
static void
send_notify(uint32_t end)
{
    lastNotifiedEnd = end;
    send_frame( ManagedString("N ") + ManagedString((int)end) + " 0" );
}

/**
 * Watch for the end of the log moving, and tell a subscribed host where it now
 * is. Polling is cheap: MicroBitLog::init returns immediately once the log is
 * initialised, so getDataLength is a mutex wait and a subtraction.
 *
 * Notices are skipped while a read is in progress, so an 'N' frame cannot land
 * in the middle of a chunk's raw bytes. Skipping one costs nothing because the
 * next notice carries the absolute end anyway.
 */
static void
notify_fiber()
{
    while(1)
    {
        uBit.sleep(BRIDGE_NOTIFY_INTERVAL);

        if (!subscribed || readInProgress)
            continue;

        uint32_t end = uBit.log.getDataLength( DataFormat::CSV );

        // Catches the log being cleared as well as grown: the host sees the
        // value go down and knows to start again.
        if (end != lastNotifiedEnd)
            send_notify(end);
    }
}

/**
 * Read a decimal number from request, starting at *pos, skipping any leading
 * spaces. Advances *pos past the digits consumed.
 *
 * @return DEVICE_OK, or DEVICE_INVALID_PARAMETER if no digits were found.
 */
static int
parse_number(const char *request, int len, int *pos, uint32_t *out)
{
    int p = *pos;
    uint32_t value = 0;
    int digits = 0;

    while (p < len && request[p] == ' ')
        p++;

    while (p < len && request[p] >= '0' && request[p] <= '9')
    {
        value = value * 10 + (uint32_t)(request[p] - '0');
        p++;
        digits++;
    }

    if (digits == 0)
        return DEVICE_INVALID_PARAMETER;

    *pos = p;
    *out = value;

    return DEVICE_OK;
}

/**
 * Send a data frame followed by its raw bytes and an end frame.
 */
static void
send_data(char type, uint32_t index, uint32_t count)
{
    send_frame( ManagedString(type) + " " + ManagedString((int)index)
                + " " + ManagedString((int)count)
                + " " + ManagedString((int)crc16(chunkBuffer, count)) );

    uBit.serial.send( chunkBuffer, count );

    send_frame("e");
}

/**
 * Handle 'R <index> <count>': read count bytes of CSV from byte offset index.
 *
 * Replies with a 'd' frame stating the offset, the byte count and the CRC of
 * the bytes, then exactly that many raw bytes with no terminator, then an 'e'
 * frame. The host finds the end of the data from the count, never from a
 * newline, because log data contains newlines of its own.
 *
 * The request is clamped rather than rejected when it runs past the end of the
 * log, so the header always describes what was actually sent.
 */
static void
handle_read(const char *request, int len)
{
    int pos = 1;
    uint32_t index;
    uint32_t count;

    if ( parse_number(request, len, &pos, &index) != DEVICE_OK ||
         parse_number(request, len, &pos, &count) != DEVICE_OK )
    {
        send_error(DEVICE_INVALID_PARAMETER);
        return;
    }

    uint32_t total = uBit.log.getDataLength( DataFormat::CSV );

    if (index >= total)
    {
        send_error(DEVICE_INVALID_PARAMETER);
        return;
    }

    if (count > BRIDGE_MAX_CHUNK)
        count = BRIDGE_MAX_CHUNK;

    if (index + count > total)
        count = total - index;

    readInProgress = true;

    int result = uBit.log.readData( chunkBuffer, index, count, DataFormat::CSV, total );

    if (result == DEVICE_OK)
        send_data('d', index, count);
    else
        send_error(result);

    readInProgress = false;
}

/**
 * Handle 'H': return the column header row.
 *
 * This reads the first row out of the log, which is the header the log was
 * started with. MicroBitLog writes a fresh header row into the data region
 * whenever the columns change, so a log whose columns changed part way through
 * has more than one, and this reply will name the wrong columns for later rows.
 *
 * Answering correctly needs the headings MicroBitLog already holds in memory,
 * which are private with no accessor. Adding one is the right fix and belongs
 * in the conversation with Lancaster. Recorded in the handoff doc.
 */
static void
handle_headers()
{
    uint32_t total = uBit.log.getDataLength( DataFormat::CSV );

    if (total == 0)
    {
        send_error(DEVICE_NO_DATA);
        return;
    }

    uint32_t count = (total < BRIDGE_MAX_CHUNK) ? total : BRIDGE_MAX_CHUNK;

    readInProgress = true;

    int result = uBit.log.readData( chunkBuffer, 0, count, DataFormat::CSV, total );

    if (result == DEVICE_OK)
    {
        // Trim to the first line, dropping any trailing carriage return.
        uint32_t n = 0;

        while (n < count && chunkBuffer[n] != '\n')
            n++;

        if (n > 0 && chunkBuffer[n-1] == '\r')
            n--;

        send_data('h', 0, n);
    }
    else
    {
        send_error(result);
    }

    readInProgress = false;
}

/**
 * Handle 'S <0|1>': subscribe to, or unsubscribe from, end-of-data notices.
 *
 * Subscribing sends one notice immediately, so the host has a baseline to
 * compare against rather than waiting for the next row to be written.
 */
static void
handle_subscribe(const char *request, int len)
{
    int pos = 1;
    uint32_t enable;

    if ( parse_number(request, len, &pos, &enable) != DEVICE_OK )
    {
        send_error(DEVICE_INVALID_PARAMETER);
        return;
    }

    subscribed = (enable != 0);

    send_frame( ManagedString("s ") + ManagedString((int)subscribed) );

    if (subscribed)
        send_notify( uBit.log.getDataLength( DataFormat::CSV ) );
}

/**
 * Handle 'M <0|1>': turn log mirroring on or off.
 *
 * Mirroring writes each logged row to the serial port as plain text, outside
 * any frame. It is here as test apparatus: it is the easiest way to put
 * non-protocol traffic on the wire and confirm a client can still pick the
 * frames out.
 */
static void
handle_mirror(const char *request, int len)
{
    int pos = 1;
    uint32_t enable;

    if ( parse_number(request, len, &pos, &enable) != DEVICE_OK )
    {
        send_error(DEVICE_INVALID_PARAMETER);
        return;
    }

    mirroring = (enable != 0);
    uBit.log.setSerialMirroring(mirroring);

    send_frame( ManagedString("m ") + ManagedString((int)mirroring) );
}

/**
 * Send count bytes as ASCII hex, in one frame.
 *
 * Built into a fixed buffer rather than by concatenating ManagedStrings,
 * which would allocate and copy once per byte.
 */
static void
send_hex(const uint8_t *data, uint32_t count)
{
    static const char digits[] = "0123456789ABCDEF";

    for (uint32_t i = 0; i < count; i++)
    {
        hexBuffer[i*2]     = digits[ data[i] >> 4 ];
        hexBuffer[i*2 + 1] = digits[ data[i] & 0x0F ];
    }

    hexBuffer[count*2] = 0;

    send_frame( ManagedString("x ") + ManagedString((const char *)hexBuffer) );
}

/**
 * Handle 'D': report what the device itself believes about the log.
 *
 * This exists because MY_DATA.HTM on the MICROBIT drive is not a reliable
 * witness. The host caches the mass storage sectors, and the drive is only
 * remounted by MicroBitLog::_setVisibility, which ordinary logging never
 * triggers. A dump of that file can therefore be stale, or a mixture of two
 * points in time. Everything here is read live from DAPLink's flash over I2C,
 * so it cannot be stale.
 *
 * Sent at boot as well as on request, so the state immediately after a reflash
 * can be seen without having to catch it.
 */
static void
send_diagnostics()
{
    uint32_t total = uBit.log.getDataLength( DataFormat::CSV );

    send_frame( ManagedString("i ") + ManagedString((int)uBit.log.isPresent())
                + " " + ManagedString((int)uBit.log.isFull())
                + " " + ManagedString((int)total) );

    if (total == 0)
    {
        send_frame("x");
        return;
    }

    uint32_t count = (total < BRIDGE_MAX_CHUNK) ? total : BRIDGE_MAX_CHUNK;

    readInProgress = true;

    int result = uBit.log.readData( chunkBuffer, 0, count, DataFormat::CSV, total );

    if (result == DEVICE_OK)
        send_hex(chunkBuffer, count);
    else
        send_error(result);

    readInProgress = false;
}

/**
 * Send filler bytes, to prime DAPLink's serial buffer.
 *
 * DAPLink holds serial bytes from the nRF52 in a buffer shared between its CDC
 * port and WebUSB. A WebUSB client does not see a short, isolated reply: a
 * program that chatters gets through, one that only speaks when spoken to does
 * not, and a client would conclude the device was dead.
 *
 * So the device fills the buffer itself, and the filler goes FIRST. When the
 * buffer overflows the oldest bytes are discarded, which was measured on the
 * client side: a reply sent ahead of 2KB of filler never arrived, while filler
 * sent after it did. Anything we send before the reply is therefore expendable
 * and the reply, being newest, cannot be the thing that gets dropped.
 *
 * The filler carries no record separator, so a client scanning for frames
 * discards it without needing to know it exists.
 *
 * BRIDGE_PRIME_BYTES is not tuned against a measured buffer size. Raising it is
 * safe in this order but buys nothing beyond the size of the buffer; if a client
 * still sees no reply, the fault is not here.
 */
static void
send_padding()
{
    static uint8_t filler[64];
    static bool ready = false;

    if (!ready)
    {
        for (int i = 0; i < 62; i++)
            filler[i] = '.';

        // Break it into lines, so a human watching a terminal sees something
        // legible rather than one unbroken block.
        filler[62] = '\r';
        filler[63] = '\n';

        ready = true;
    }

    for (int sent = 0; sent < BRIDGE_PRIME_BYTES; sent += (int)sizeof(filler))
        uBit.serial.send( filler, sizeof(filler) );
}

static void
handle_hello(const char *request, int len)
{
    int flags = 0;

    if (uBit.log.isPresent())
        flags |= BRIDGE_FLAG_PRESENT;

    if (uBit.log.isFull())
        flags |= BRIDGE_FLAG_FULL;

    if (mirroring)
        flags |= BRIDGE_FLAG_MIRRORING;

    if (subscribed)
        flags |= BRIDGE_FLAG_SUBSCRIBED;

    if (logging)
        flags |= BRIDGE_FLAG_LOGGING;

    // '?' primes by default, because the first hello of a session is the one
    // that has to get through. '? 0' skips the filler, for every hello after
    // that, once the buffer is already warm.
    //
    // The filler goes first and the reply last. Sending the reply first loses
    // it: the buffer discards its oldest bytes, and 2KB of filler behind a ten
    // byte reply is exactly how to throw that reply away.
    int pos = 1;
    uint32_t pad;

    if ( parse_number(request, len, &pos, &pad) != DEVICE_OK || pad != 0 )
        send_padding();

    send_frame( ManagedString("! ") + ManagedString(BRIDGE_VERSION) + " " + ManagedString(flags) );
}

static void
handle_request(const char *request, int len)
{
    // A leading separator is accepted so hand-typed and generated requests
    // both work.
    if (len > 0 && request[0] == (char)BRIDGE_RS)
    {
        request++;
        len--;
    }

    if (len == 0)
        return;

    switch (request[0])
    {
        case '?':
            handle_hello(request, len);
            break;

        case 'L':
        {
            uint32_t length = uBit.log.getDataLength( DataFormat::CSV );
            send_frame( ManagedString("l 2 ") + ManagedString((int)length) );
            break;
        }

        case 'R':
            handle_read(request, len);
            break;

        case 'H':
            handle_headers();
            break;

        case 'S':
            handle_subscribe(request, len);
            break;

        case 'M':
            handle_mirror(request, len);
            break;

        case 'D':
            send_diagnostics();
            break;

        default:
            send_error(DEVICE_INVALID_PARAMETER);
            break;
    }
}

void
serial_bridge_test()
{
    uBit.serial.setTxBufferSize(BRIDGE_TX_BUFFER);

    uBit.messageBus.listen( DEVICE_ID_BUTTON_A, DEVICE_BUTTON_EVT_CLICK, onButtonA );
    uBit.messageBus.listen( DEVICE_ID_BUTTON_B, DEVICE_BUTTON_EVT_CLICK, onButtonB );

    // Registering this listener is what makes a two button press behave. MicroBit
    // watches for it and switches buttons A and B to DEVICE_BUTTON_SIMPLE_EVENTS,
    // handing click detection to the multibutton, so pressing both no longer
    // fires a start and a stop on the way to the clear. See MicroBit.cpp:328.
    uBit.messageBus.listen( DEVICE_ID_BUTTON_AB, DEVICE_BUTTON_EVT_CLICK, onButtonAB );

    create_fiber( log_row_fiber );
    create_fiber( notify_fiber );

    send_frame( ManagedString("! ") + ManagedString(BRIDGE_VERSION) + " 0" );

    // Report the log's state as found, before anything in this program has had
    // a chance to change it.
    send_diagnostics();

    int len = 0;
    bool overflow = false;

    while(1)
    {
        int c = uBit.serial.read(SYNC_SLEEP);

        if (c < 0)
            continue;

        if (c != '\r' && c != '\n')
        {
            if (len < BRIDGE_MAX_REQUEST)
                requestBuffer[len++] = (char)c;
            else
                overflow = true;

            continue;
        }

        // End of a request line.
        if (len > 0)
        {
            uBit.display.image.setPixelValue(2, 2, 255);

            if (overflow)
                send_error(DEVICE_INVALID_PARAMETER);
            else
                handle_request(requestBuffer, len);

            uBit.sleep(50);
            uBit.display.image.setPixelValue(2, 2, 0);
        }

        len = 0;
        overflow = false;
    }
}
