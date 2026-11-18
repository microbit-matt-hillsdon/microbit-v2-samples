#include "MicroBit.h"
#include "samples/Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();

    // The demo build. Swap for serial_bridge_test() to get the development
    // build, which adds the benchmark, fill, diagnostic and contention
    // simulation commands.
    data_log_demo();

    microbit_panic( 999 );
}

