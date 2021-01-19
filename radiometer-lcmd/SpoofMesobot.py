#!/usr/bin/env python3
"""flowmeter_can_lcmd - LCM daemon for flow meter."""

import select
import serial
import struct
import time

class RadiometerDaemon:
    """LCM daemon for radiometer."""

    def __init__(self, dev='/dev/ttyUSB0', br = 115200):
        """Define CAN and LCM interfaces, and subscribe to input."""
        self.serial = serial.Serial(dev, baudrate=br, timeout=0.1)
        self.hdrpkt = struct.Struct('H2L') # 2 unsigned long bytes (ISR clock cycles, LOG clock cycles)
        self.hbpkt = struct.Struct('H5L') # 5 unsigned long bytes (UTC, Pulse count, nsHI, irradiance, end token)

        self.filename_data = time.strftime("Speedtest_%Y%m%d-%H.%M_Data")
        self.f = open(self.filename_data, "w")
        self.filename_profiler = time.strftime("Speedtest_%Y%m%d-%H.%M_Profiler")
        self.fp = open(self.filename_profiler, "w")

    def serial_handler(self):
        """Receive data on serial port and send on LCM."""
        hdr = self.serial.read(2)
        if hdr is None:
            print('tried to handle empty serial message queue')
        elif len(hdr) == 2 and int.from_bytes(hdr,"little") == 0xFF: # header
            rx = self.serial.read(10)
            (h, ISR, LOG) = self.hdrpkt.unpack(rx)
            #write the data to file here
            self.fp.write("{0}\t{1}\n".format(ISR, LOG))
        elif len(hdr) == 2 and int.from_bytes(hdr,"little") == 0xFE: # heartbeat
            rx = self.serial.read(22)
            print('rx heartbeat message')
        else:
            dat = int.from_bytes(hdr,"little")
            self.f.write(dat)
        # msg = self.serial.read(8)
        # self.f.write("{0}\t{1}\t{2}\t{3}\n".format(*self.pkt.unpack(msg)))

    def connect(self):
        """Connect serial to LCM and loop with epoll."""
        epoll = select.epoll()
        epoll.register(self.serial.fileno(), select.EPOLLIN)
        try:
            while True:
                for fileno, _events in epoll.poll(1):
                    if fileno == self.serial.fileno():
                        self.serial_handler()
        except (KeyboardInterrupt, SystemExit):
            print('stopped by user')
        finally:
            epoll.unregister(self.serial.fileno())
            epoll.close()


def main(dev="/dev/ttyUSB0", br = 115200, verbose=0):
    """Run as a daemon."""
    bridge = RadiometerDaemon(dev)
    bridge.connect()


if __name__ == "__main__":
    import argparse
    P = argparse.ArgumentParser(description="LCM daemon for radiometer")
    P.add_argument('-v', '--verbose', action='count', default=0,
                   help='display verbose output')
    P.add_argument('-V', '--version', action='version',
                   version='%(prog)s 0.0.1',
                   help='display version information and exit')
    P.add_argument('-dev', help='the serial device to use', default="/dev/ttyUSB0")
    P.add_argument('-br', help='the baudrate to use', default=115200)
    A = P.parse_args()
    main(**A.__dict__)
