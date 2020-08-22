#!/usr/bin/python

import time, sys

from arbotix_python.arbotix import ArbotiX
from arbotix_python.ax12 import *


class Terminal(ArbotiX):

    def __init__(self, port = "/dev/ttyUSB0", baud = 115200):
        # start
        ArbotiX.__init__(self, port, baud)  

        while True:
            self.setPosition(1, 1050)
            #self.setPosition(2, 1050)
            self.setPosition(3, 1050)

            self.setPosition(4, 1050)
            #self.setPosition(5, 1050)
            self.setPosition(6, 1050)

            self.setPosition(7, 1050)
            #self.setPosition(8, 1050)
            self.setPosition(9, 1050)

            self.setPosition(10, 1050)
            #self.setPosition(11, 1050)
            self.setPosition(12, 1050)

            time.sleep(0.5)

            self.setPosition(1, 2050)
            #self.setPosition(2, 1050)
            self.setPosition(3, 2050)

            self.setPosition(4, 2050)
            #self.setPosition(5, 1050)
            self.setPosition(6, 2050)

            self.setPosition(7, 2050)
            #self.setPosition(8, 1050)
            self.setPosition(9, 2050)

            self.setPosition(10, 2050)
            #self.setPosition(11, 1050)
            self.setPosition(12, 2050)

            time.sleep(0.5)

if __name__ == "__main__":
    try:
        if len(sys.argv) > 2:
            t = Terminal(sys.argv[1], int(sys.argv[2]))
        elif len(sys.argv) > 1:
            t = Terminal(sys.argv[1])
        else:
            t = Terminal()
    except KeyboardInterrupt:
        print "\nExiting..."