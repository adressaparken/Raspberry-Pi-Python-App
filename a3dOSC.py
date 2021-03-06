#!/usr/bin/python

# Version:      0.1
# Last changed: 06/08/17

import sys
import signal
import time
import random
import socket
from pythonosc import osc_message_builder
from pythonosc import udp_client
from a3dLogger import a3dLogger

class OSCHandler():

    def __init__(self, ip, port, log_level):

        self.logger = a3dLogger()
        self.logger.set_level(log_level)

        self.client = udp_client.SimpleUDPClient(ip, port)

    def send_message( self, address, msg ):
        self.client.send_message( address, msg )


if __name__ == '__main__':

    # init stuff
    osc = OSCHandler( "192.168.1.255", 5005 )

    # main program loop
    for x in range(10):
        osc.send_message("/filter", random.random())


        time.sleep(1)

    # cleanup
    print( 'Closing program!' )
    sys.exit(0)
