#include "MicroBit.h"
#include "samples/Tests.h"

MicroBit uBit;

int main()
{
    uBit.init();

    data_logging_csv_from_char_arr();

    microbit_panic(999);
}
