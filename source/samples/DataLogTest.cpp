#include "MicroBit.h"
#include "Tests.h"

//
// Logs temperature and light level once per second into the micro:bit's
// on-board log, which appears as MY_DATA.HTM on the MICROBIT drive.
//
// Button A clears the log and starts again.
//

void
data_log_test()
{
    // Put a timestamp column at the front of every row, counted in seconds.
    uBit.log.setTimeStamp( TimeStampFormat::Seconds );

    // Also print each row to the serial port, so we can watch it live.
    uBit.log.setSerialMirroring( true );

    while(1)
    {
        if ( uBit.buttonA.isPressed() )
        {
            uBit.log.clear();
            uBit.display.printChar('C');
            uBit.sleep(500);
            uBit.display.clear();
        }

        if ( uBit.log.isFull() )
        {
            uBit.display.printChar('F');
            uBit.sleep(1000);
            continue;
        }

        int temperature = uBit.thermometer.getTemperature();
        int light = uBit.display.readLightLevel();

        uBit.log.beginRow();
        uBit.log.logData( "temperature", ManagedString(temperature) );
        uBit.log.logData( "light", ManagedString(light) );
        uBit.log.endRow();

        DMESG("LOGGED: temp %d light %d", temperature, light);

        // Blink one pixel so we can see it is still running.
        uBit.display.image.setPixelValue(2, 2, 255);
        uBit.sleep(100);
        uBit.display.image.setPixelValue(2, 2, 0);
        uBit.sleep(900);
    }
}
