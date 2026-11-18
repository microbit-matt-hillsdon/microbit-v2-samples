#include "MicroBit.h"
#include "Tests.h"

//
// A demo of reading the data log out of a running program over the serial link.
//
// The logging half is a faithful C++ translation of the MakeCode program a user
// would really write:
//
//     datalogger.setColumnTitles("x", "y", "z")
//     loops.everyInterval(100, () => {
//         if (logging) datalogger.log(x, y, z)
//     })
//     input.onButtonPressed(A,  () => { logging = true;  showIcon(Yes)   })
//     input.onButtonPressed(B,  () => { logging = false; showIcon(No)    })
//     input.onButtonPressed(AB, () => { logging = false; showIcon(Skull)
//                                       deleteLog(); setColumnTitles(...) })
//     datalogger.onLogFull(     () => { logging = false; showLeds(all)   })
//
// With a "Time (seconds)" column, which MakeCode's datalogger includes by
// default without being asked.
//
// Clearing the log on reflash is CODAL's own behaviour, from
// CONFIG_MICROBIT_ERASE_USER_DATA_ON_REFLASH, which defaults to 1. If a flash
// leaves the old log in place, check that codal.json has not turned it off.
//
// The serial protocol is bolted onto that program rather than the other way
// round, which is the right way round for judging whether the two can coexist.
// The display belongs to the logging program: this file shows nothing of its
// own on it, because a production bridge could not take the display from the
// user's program either.
//
// This is the demo build. For the development build, with benchmark, fill,
// diagnostic and contention-simulation commands, see SerialBridgeTest.cpp.
//
// The protocol here is only what a real client needs, and is deliberately kept
// to the shape of the Bluetooth utility service so one host client can sit over
// either transport. That service (inc/bluetooth/MicroBitUtilityTypes.h) answers
// exactly two requests:
//
//     requestTypeLogLength    ->  4 byte length
//     requestTypeLogRead      ->  format, index, batchlen, length -> log bytes
//
// which are 'L' and 'R' below. 'H' and the 'S'/'N' live-update pair have no
// Bluetooth equivalent — that service has no notification, so a Bluetooth
// client polls the length instead. They are kept because the live view is the
// point of the demo.
//
// One deliberate divergence: the Bluetooth read carries a format argument and
// 'R' here does not, because every caller wants CSV. If the HTML forms are ever
// needed, 'R' grows a leading format argument and matches exactly.
//
// Requests are whole lines, so press Enter after each one in a terminal. A
// leading 0x1E is accepted but not required.
//

#define DEMO_RS                 0x1E
#define DEMO_VERSION            1
#define DEMO_MAX_REQUEST        32
#define DEMO_MAX_CHUNK          64
#define DEMO_TX_BUFFER          128
#define DEMO_NOTIFY_INTERVAL    250
#define DEMO_PRIME_BYTES        2048
#define DEMO_LOG_INTERVAL       100

// Flags reported in the hello reply.
#define DEMO_FLAG_PRESENT       0x01
#define DEMO_FLAG_FULL          0x02
#define DEMO_FLAG_LOGGING       0x04

static char     requestBuffer[DEMO_MAX_REQUEST];
static uint8_t  chunkBuffer[DEMO_MAX_CHUNK];
static bool     subscribed = false;
static bool     readInProgress = false;
static bool     logging = false;
static uint32_t lastNotifiedEnd = 0;

// Display states, standing in for MakeCode's IconNames.
static const char *ICON_STOPPED =
    "1,0,0,0,1\n"
    "0,1,0,1,0\n"
    "0,0,1,0,0\n"
    "0,1,0,1,0\n"
    "1,0,0,0,1\n";

static const char *ICON_LOGGING =
    "0,0,0,0,0\n"
    "0,0,0,0,1\n"
    "0,0,0,1,0\n"
    "1,0,1,0,0\n"
    "0,1,0,0,0\n";

static const char *ICON_CLEARED =
    "0,1,1,1,0\n"
    "1,0,1,0,1\n"
    "1,1,1,1,1\n"
    "0,1,1,1,0\n"
    "0,1,1,1,0\n";

static const char *ICON_FULL =
    "1,1,1,1,1\n"
    "1,1,1,1,1\n"
    "1,1,1,1,1\n"
    "1,1,1,1,1\n"
    "1,1,1,1,1\n";

static void
show_icon(const char *icon)
{
    uBit.display.image.paste( MicroBitImage(icon), 0, 0, 0 );
}

/**
 * Set up the log the way MakeCode's datalogger does by default: a timestamp
 * column in seconds, then the named columns, before anything is logged.
 *
 * Both have to happen while the log is empty. MicroBitLog adds the timestamp
 * heading at the first endRow and only puts it *first* when dataStart ==
 * dataEnd (MicroBitLog.cpp:560); against a log that already holds rows it is
 * appended instead, and the columns come out as "x,y,z,Time (seconds)".
 *
 * Name the columns without writing a row, as datalogger.setColumnTitles does.
 *
 * MicroBitLog has no equivalent call and does not need one: logData with empty
 * values registers the headings, and _endRow suppresses the data row when every
 * value is empty (MicroBitLog.cpp:645-660). So this writes the header line and
 * nothing else.
 *
 * Must run while the log is empty, or MicroBitLog appends the columns after
 * whatever is already there rather than starting a fresh header.
 */
static void
set_column_titles()
{
    uBit.log.setTimeStamp( TimeStampFormat::Seconds );

    uBit.log.beginRow();
    uBit.log.logData( "x", ManagedString::EmptyString );
    uBit.log.logData( "y", ManagedString::EmptyString );
    uBit.log.logData( "z", ManagedString::EmptyString );
    uBit.log.endRow();
}

static void
onButtonA(Event)
{
    logging = true;
    show_icon(ICON_LOGGING);
}

static void
onButtonB(Event)
{
    logging = false;
    show_icon(ICON_STOPPED);
}

/**
 * A and B together: stop, throw the log away, and name the columns again.
 *
 * Logging is stopped first so the logging fiber cannot append a row between the
 * erase and the user seeing an empty log.
 *
 * A subscribed host is not told directly. notify_fiber sees the end of the data
 * drop and sends the usual 'N', which has always meant "the data now ends here"
 * rather than "the data grew", so a client that handles a shrinking end needs
 * no new message for this.
 */
static void
onButtonAB(Event)
{
    logging = false;

    uBit.log.clear();
    set_column_titles();

    show_icon(ICON_CLEARED);
}

/**
 * The log has filled.
 *
 * MicroBitLog raises this from _logString when it runs out of space
 * (MicroBitLog.cpp:744). Worth knowing for the client: the log stops accepting
 * rows and there is no "logging has stopped but the live view continues" state
 * anywhere in CODAL. The length simply stops moving.
 */
static void
onLogFull(Event)
{
    logging = false;
    show_icon(ICON_FULL);
}

/**
 * loops.everyInterval(100, ...) — one accelerometer row per interval.
 */
static void
log_row_fiber()
{
    while(1)
    {
        if ( logging && !uBit.log.isFull() )
        {
            uBit.log.beginRow();
            uBit.log.logData( "x", ManagedString(uBit.accelerometer.getX()) );
            uBit.log.logData( "y", ManagedString(uBit.accelerometer.getY()) );
            uBit.log.logData( "z", ManagedString(uBit.accelerometer.getZ()) );
            uBit.log.endRow();
        }

        uBit.sleep(DEMO_LOG_INTERVAL);
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
    uBit.serial.send( ManagedString((char)DEMO_RS) + body + "\r\n" );
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
 * headingsChanged are private with no public accessor. Harmless here, because
 * this program names its columns once and never changes them.
 */
static void
send_notify(uint32_t end)
{
    lastNotifiedEnd = end;
    send_frame( ManagedString("N ") + ManagedString((int)end) + " 0" );
}

/**
 * Watch for the end of the log moving, and tell a subscribed host where it now
 * is. Notices are skipped while a read is in progress so an 'N' frame cannot
 * land in the middle of a chunk's raw bytes; skipping one costs nothing because
 * the next carries the absolute end anyway.
 */
static void
notify_fiber()
{
    while(1)
    {
        uBit.sleep(DEMO_NOTIFY_INTERVAL);

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
 * The Bluetooth equivalent is requestTypeLogRead, which takes the same index
 * and batch length plus the whole-file length from a preceding length request.
 * MicroBitLog::readData takes that length too, so the two are the same call.
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

    // Distinguished from a malformed request, so a client can tell "you sent me
    // rubbish" from "there is nothing here yet".
    if (total == 0 || index >= total)
    {
        send_error(DEVICE_NO_DATA);
        return;
    }

    if (count > DEMO_MAX_CHUNK)
        count = DEMO_MAX_CHUNK;

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
 * Reads the first row out of the log, which is the header this program wrote at
 * startup. That is correct here because the columns never change. It would not
 * be correct in general: MicroBitLog writes a fresh header row into the data
 * region whenever the columns change, so a log whose columns changed part way
 * through has more than one. See the handoff doc.
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

    uint32_t count = (total < DEMO_MAX_CHUNK) ? total : DEMO_MAX_CHUNK;

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
 * Send filler bytes, to prime DAPLink's serial buffer.
 *
 * DAPLink holds serial bytes from the nRF52 in a buffer shared between its CDC
 * port and WebUSB, and a short, isolated reply does not reach a WebUSB client.
 * A program that chatters gets through; one that only speaks when spoken to
 * does not, and a client would conclude the device was dead.
 *
 * So the device fills the buffer itself, and the filler goes FIRST. When the
 * buffer overflows the oldest bytes are discarded, measured on the client side:
 * a reply sent ahead of 2KB of filler never arrived, while filler sent after it
 * did. Anything before the reply is therefore expendable, and the reply, being
 * newest, cannot be what gets dropped.
 *
 * The filler carries no record separator, so a client scanning for frames
 * discards it without needing to know it exists.
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

    for (int sent = 0; sent < DEMO_PRIME_BYTES; sent += (int)sizeof(filler))
        uBit.serial.send( filler, sizeof(filler) );
}

/**
 * Handle '?': reply with the protocol version, the flags, and the log mode.
 *
 * The third number is MICROBIT_LOG_MODE, the CODAL build option that picks the
 * HTML header written at the front of the log — 0 the data.microbit.org
 * experience, 1 the basic offline one, 2 next gen, 3 datatool. It says which
 * display experience this hex was built for.
 *
 * It is reported because the header itself is unreachable over this protocol:
 * 'R' reads DataFormat::CSV, which starts below the header, so a client that
 * never opens MY_DATA.HTM has no other way to know. See
 * docs/datalog-datatool-header.md.
 *
 * The value is compiled in and cannot change while the program runs, so it is a
 * separate field rather than a flag bit: it describes the build, not the state.
 */
static void
handle_hello(const char *request, int len)
{
    int flags = 0;

    if (uBit.log.isPresent())
        flags |= DEMO_FLAG_PRESENT;

    if (uBit.log.isFull())
        flags |= DEMO_FLAG_FULL;

    if (logging)
        flags |= DEMO_FLAG_LOGGING;

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

    send_frame( ManagedString("! ") + ManagedString(DEMO_VERSION)
                + " " + ManagedString(flags)
                + " " + ManagedString(MICROBIT_LOG_MODE) );
}

static void
handle_request(const char *request, int len)
{
    // A leading separator is accepted so hand-typed and generated requests
    // both work.
    if (len > 0 && request[0] == (char)DEMO_RS)
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

        default:
            send_error(DEVICE_INVALID_PARAMETER);
            break;
    }
}

void
data_log_demo()
{
    uBit.serial.setTxBufferSize(DEMO_TX_BUFFER);

    uBit.messageBus.listen( DEVICE_ID_BUTTON_A, DEVICE_BUTTON_EVT_CLICK, onButtonA );
    uBit.messageBus.listen( DEVICE_ID_BUTTON_B, DEVICE_BUTTON_EVT_CLICK, onButtonB );

    // Registering a listener on the multibutton is what makes a two button
    // press behave: MicroBit watches for it and switches A and B to
    // DEVICE_BUTTON_SIMPLE_EVENTS, handing click detection to the multibutton,
    // so pressing both no longer fires a start and a stop on the way to the
    // clear. See MicroBit.cpp:328.
    uBit.messageBus.listen( DEVICE_ID_BUTTON_AB, DEVICE_BUTTON_EVT_CLICK, onButtonAB );
    uBit.messageBus.listen( MICROBIT_ID_LOG, MICROBIT_LOG_EVT_LOG_FULL, onLogFull );

    set_column_titles();

    logging = false;
    show_icon(ICON_STOPPED);

    create_fiber( log_row_fiber );
    create_fiber( notify_fiber );

    // The boot hello carries the log mode too, so the three replies all have
    // the same shape. A WebUSB client will probably miss this one — it goes out
    // with no filler in front of it — but a terminal sees it.
    send_frame( ManagedString("! ") + ManagedString(DEMO_VERSION) + " 0 "
                + ManagedString(MICROBIT_LOG_MODE) );

    int len = 0;
    bool overflow = false;

    while(1)
    {
        int c = uBit.serial.read(SYNC_SLEEP);

        if (c < 0)
            continue;

        if (c != '\r' && c != '\n')
        {
            if (len < DEMO_MAX_REQUEST)
                requestBuffer[len++] = (char)c;
            else
                overflow = true;

            continue;
        }

        // End of a request line.
        if (len > 0)
        {
            if (overflow)
                send_error(DEVICE_INVALID_PARAMETER);
            else
                handle_request(requestBuffer, len);
        }

        len = 0;
        overflow = false;
    }
}
