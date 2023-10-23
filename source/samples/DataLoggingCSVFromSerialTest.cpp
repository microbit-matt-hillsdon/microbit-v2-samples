#include "MicroBit.h"
#include "Tests.h"

void data_logging_csv_from_serial()
{
    uBit.serial.setRxBufferSize(128);
    while (true)
    {
        while (true)
        {
            if (uBit.buttonA.wasPressed())
            {
                uBit.display.scroll("L");
                break;
            }
            if (uBit.buttonB.wasPressed())
            {
                uBit.display.scroll("C");
                uBit.log.clear(false);
            }
            uBit.sleep(10);
        }
        while (true)
        {
            ManagedString line = uBit.serial.readUntil("\n", SerialMode::SYNC_SPINWAIT);
            if (line.charAt(line.length() - 1) == '\r')
            {
                line = line.substring(0, line.length() - 1);
            }
            if (line == "EOF")
            {
                uBit.serial.send("OK\r\n");
                break;
            }
            else
            {
                // Skip the headers/rows layer and just log directly
                uBit.log.logString(line + '\n');
            }
            uBit.sleep(1);
        }
    }
}