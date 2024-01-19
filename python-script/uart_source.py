import serial
import sys
import queue
import os
import errno
import time


FIFO = 'serial.fifo'

# four byte header to split messages
# one byte of tag ID
# four bytes of distance - 15000 = 1.5 * 10000 = 0x00_00_3a_98
# sent LSB first
EXPECTED_DATA = bytes((0xFF, 0xFF, 0xFF, 0xFF, 0xFA, 0x98, 0x3a, 0x0, 0x0))


def init_fifo(file_name: str):
    try:
        os.mkfifo(file_name)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def serial_receive(port: str, q: queue.Queue=None):
    # init_fifo(FIFO)

    while True:
        with serial.Serial(port, 115200, timeout=2) as ser:
            line = ser.readline().decode("utf-8")

            # with open(FIFO, 'w') as f:
            #     f.write(line)

            if line:
                data = line.split(",")

                if len(data) == 3:
                    data = (float(data[0]), float(data[1]), float(data[2]))
                    if q.qsize() < 24:
                        q.put(data)
