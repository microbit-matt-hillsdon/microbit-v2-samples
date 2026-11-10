#include "MicroBit.h"
#include "samples/Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();

    serial_bridge_test();

    microbit_panic( 999 );
}

