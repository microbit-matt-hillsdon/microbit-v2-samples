#include "MicroBit.h"
#include "samples/Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();

    data_logging_timeseries();

    microbit_panic( 999 );
}

