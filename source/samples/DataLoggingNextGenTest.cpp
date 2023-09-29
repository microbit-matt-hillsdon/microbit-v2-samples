#include "MicroBit.h"
#include "Tests.h"

void data_logging_timeseries_next_gen()
{
    uBit.log.setTimeStamp(TimeStampFormat::None);
    bool logging = false;
    int activityLevel = 1;
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
            if (microbit_random(101) >= 99) {
                activity = microbit_random(5);
                if (activity == 0)
                {
                    activity += 1;
                }
                int change = microbit_random(2);
                if (change == 0) 
                {
                activityLevel--;
                } 
                else 
                {
                activityLevel++;
                }
                if (activityLevel < 0) 
                {
                    activityLevel = 0;
                }
                if (activityLevel > 3) 
                {
                    activityLevel = 3;
                } 
            }
            uBit.log.beginRow();
            uBit.log.logData("Activity level", activityLevel);
            uBit.log.logData("Activity", activity);
            uBit.log.endRow();
        }
        uBit.sleep(1);
    }
}