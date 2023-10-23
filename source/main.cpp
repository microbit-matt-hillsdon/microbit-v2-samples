#include "MicroBit.h"
#include "samples/Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();

    data_logging_csv_from_serial();

    microbit_panic(999);
}
