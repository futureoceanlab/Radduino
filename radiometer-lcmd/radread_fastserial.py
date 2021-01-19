#!/usr/bin/env python3
"""flowmeter_can_lcmd - LCM daemon for flow meter."""

import select
import serial
import struct
import time
import os
import threading
import queue

class RadiometerDaemon:
    """LCM daemon for radiometer."""

    def __init__(self, dev='/dev/ttyUSB0', br = 115200, rootfolder = '~/Documents'):
        """Define CAN and LCM interfaces, and subscribe to input."""
        self.serial = serial.Serial(dev, baudrate=br, timeout=None)
        self.hpkt = struct.Struct('3H')
        self.startfolder = os.getcwd();
        foldername = os.path.join(os.path.expanduser(rootfolder),time.strftime("Speedtest_%Y%m%d-%H.%M"))
        os.mkdir(foldername)
        os.chdir(foldername)
        self.pfname = "ping.bin"
        self.pf = open(self.pfname, "wb")
        self.hfname = "heartbeat.txt"
        self.hf = open(self.hfname,"w")
        print("baudrate {0} on dev {1}".format(br, dev))

    def serial_handler(self):
        """Receive data on serial port and send on LCM."""
        hdr = self.serial.read(2)
        if hdr is None or len(hdr) == 1: 
            print('tried to handle empty serial message queue')
        elif hdr[1] == 0xfd: #heartbeat
            hmsg = self.serial.read(6)
            self.hf.write("{0}\t{1}\t{2}\n".format(*self.hpkt.unpack(hmsg)))
        elif hdr[1] == 0xfa: #serial message
            msglen = hdr[0]
            sermsg = self.serial.read(msglen)
            print(sermsg.decode())
        else: #ping data
            self.pf.write(hdr)
            
        # elif len(hdr) == 2 and hdr[1] == int('fc', 16):
        #     rx = self.serial.read(6)
        #     (u, c, d) = self.pkt.unpack(rx)
        #     #write the data to file here
        #     self.f.write("{0}\t{1}\t{2}\n".format(u, c, d))
        # elif len(hdr) == 2 and hdr[1] == int('fd', 16):
        #     print('rx heartbeat message')
        # else:
        #     print("unknown header: {0}".format(hdr))
        #     self.serial.flushInput()
        # msg = self.serial.read(8)
        # self.f.write("{0}\t{1}\t{2}\t{3}\n".format(*self.pkt.unpack(msg)))

    def connect(self, q):
        """Connect serial to LCM and loop with epoll."""
        epoll = select.epoll()
        epoll.register(self.serial.fileno(), select.EPOLLIN)
        cont = True
        while cont:
            for fileno, _events in epoll.poll(1):
                if fileno == self.serial.fileno():
                    self.serial_handler()
            if not(q.empty()):
                print("got a message in the queue")
                msg = q.get()
                if msg is None:
                    msg = " "
                if msg == "quit":
                    cont = False
                else:
                    self.serial.write(str.encode(msg))
        epoll.unregister(self.serial.fileno())
        epoll.close()
        print("shutting down daemon")


def main(dev="/dev/ttyUSB0", br = 38400, verbose=0):
    """Run as a daemon."""
    bridge = RadiometerDaemon(dev, br)
    q = queue.Queue()
    daemon = threading.Thread(name="daemon", target=bridge.connect, args=(q,))
    #bridge.connect()
    daemon.start()
    try:
        while True:
            val = input("Send message to radiometer: ")
            q.put(val)
    except(KeyboardInterrupt, SystemExit):
        q.put("quit")




if __name__ == "__main__":
    import argparse
    P = argparse.ArgumentParser(description="LCM daemon for radiometer")
    P.add_argument('-v', '--verbose', action='count', default=0,
                   help='display verbose output')
    P.add_argument('-V', '--version', action='version',
                   version='%(prog)s 0.0.1',
                   help='display version information and exit')
    P.add_argument('-dev', help='the serial device to use', default="/dev/ttyUSB0")
    P.add_argument('-br', help='the baudrate to use', default=38400)
    A = P.parse_args()
    main(**A.__dict__)
