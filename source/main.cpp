#include "MicroBit.h"
#include "samples/Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();

    data_logging_timeseries_next_gen();

    microbit_panic( 999 );
}

