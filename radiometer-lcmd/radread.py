#!/usr/bin/env python3
"""flowmeter_can_lcmd - LCM daemon for flow meter."""

import select
import serial
import struct
import time

class RadiometerDaemon:
    """LCM daemon for radiometer."""

    def __init__(self, dev='/dev/ttyUSB1', prefix='RAD'):
        """Define CAN and LCM interfaces, and subscribe to input."""
        self.serial = serial.Serial(dev, baudrate=115200, timeout=0.1)
        self.prefix = prefix
        self.pkt = struct.Struct('!3H')

        self.subscriptions = []

    def serial_handler(self):
        """Receive data on serial port and send on LCM."""
        hdr = self.serial.read(2)
        if hdr is None:
            print('tried to handle empty serial message queue')
        elif len(hdr) == 2 and hdr[0] == int('fc', 16):
            rx = self.serial.read(6)
            (u, c, d) = self.pkt.unpack(rx)
            #write the data to file here
        elif len(hdr) == 2 and hdr[0] == int('fd', 16):
            print('rx heartbeat message')
        else:
            print("unknown header: {0:x}".format(hdr))
            self.serial.flushInput()

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


def main(dev="/dev/ttyUSB1", prefix='RAD', verbose=0):
    """Run as a daemon."""
    bridge = RadiometerDaemon(dev, prefix)
    bridge.connect()


if __name__ == "__main__":
    import argparse
    P = argparse.ArgumentParser(description="LCM daemon for radiometer")
    P.add_argument('-v', '--verbose', action='count', default=0,
                   help='display verbose output')
    P.add_argument('-V', '--version', action='version',
                   version='%(prog)s 0.0.1',
                   help='display version information and exit')
    P.add_argument('dev', help='the serial device to use')
    P.add_argument('-p', '--prefix', default='RAD',
                   help='prefix to pub/sub with')
    A = P.parse_args()
    main(**A.__dict__)
