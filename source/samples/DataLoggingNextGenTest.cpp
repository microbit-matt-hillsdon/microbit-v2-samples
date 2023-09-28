#include "MicroBit.h"
#include "Tests.h"

void data_logging_timeseries_next_gen()
{
    uBit.log.setTimeStamp(TimeStampFormat::None);
    bool logging = false;
    int activityLevel = 0;
    int activity = 1;
    while (1)
    {
        if (uBit.buttonA.wasPressed())
        {
            uBit.display.scroll("L");
            logging = true;
        }
        if (uBit.buttonB.wasPressed())
        {
            uBit.display.scroll("C");
            logging = false;
            uBit.log.clear(false);
        }
        if (uBit.log.isFull())
        {
            uBit.display.scroll("F");
            logging = false;
        }
        if (logging)
        {
            activity = microbit_random(5);
            if (activity == 0)
            {
                activity += 1;
            }
            uBit.log.beginRow();
            uBit.log.logData("Activity level", microbit_random(4));
            uBit.log.logData("Activity", activity);
            uBit.log.endRow();
        }
        uBit.sleep(1);
    }
}