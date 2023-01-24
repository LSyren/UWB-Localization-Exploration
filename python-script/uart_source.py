import serial
import sys
import queue
import os
import errno
import time


FIFO = 'serial.fifo'


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

            # fifo_write = open(FIFO, 'w', 0)
            # fifo_write.write(line)

            if line:
                #data = line.split(",")
                print("line")
                if len(data) == 3:
                    data = (float(data[0]), float(data[1]), float(data[2]))
                    if q.qsize() < 24:
                        q.put(data)

