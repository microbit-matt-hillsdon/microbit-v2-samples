# Script to write to a micro:bit running the demo of the same name.
# Requires pyserial, e.g. via python3 -m pip install pyserial
from serial import PARITY_NONE, STOPBITS_ONE, Serial
import sys
from time import sleep

# Something like /dev/cu.usbmodem11202 on Mac
serial_path = sys.argv[1];
input_path = sys.argv[2];
if not serial_path or not input_path:
    raise ValueError("Missing args")

with Serial(serial_path, 115200, timeout=0.1, parity=PARITY_NONE,
            stopbits=STOPBITS_ONE, rtscts=False, dsrdtr=False)  as ser, \
            open(input_path, "rb") as fin:
  sleep(0.5)
  data = fin.readlines();
  for line in data:
      line = line.rstrip();
      print(line)
      line += b"\r\n"
      ser.write(line)
      # Don't write too fast as there's no flow control and buffers are small
      sleep(0.05)

  # Mark EOF and wait for acknowlegement
  ser.write(b"EOF\r\n")
  sleep(0.05)
  ser.read_until(b"OK\r\n")