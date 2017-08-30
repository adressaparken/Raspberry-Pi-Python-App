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

from my_logging import *

class OSCHandler():

    def __init__( self, ip, port ):
        self.client = udp_client.SimpleUDPClient( ip, port )

    def send_message( self, address, msg ):
        self.client.send_message( address, msg )

    def send_integer( self, address, msg ):
        self.client.send_integer( address, msg )

    def send_float( self, address, msg ):
        self.client.send_float( address, msg )


if __name__ == '__main__':

    # init stuff
    osc = OSCHandler( "192.168.1.255", 5005 )

    # main program loop
    for x in range(10):
        osc.send_message("/filter", random.random())


        time.sleep(1)

    # cleanup
    log_info( 'Closing program!' )
    sys.exit(0)
