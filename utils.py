#!/usr/bin/python

import sys
import signal
import time
import socket



#------------------------------------------/
#---/get local ip address
def get_ip_address( server_ip ):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.connect((server_ip, 80))
    ip = s.getsockname()[0]
    s.close()
    return ip

#------------------------------------------/
#---/ get current time in milliseconds
def current_milli_time():
    return int(round(time.time() * 1000))

#------------------------------------------/
#---/get RPi ID
def get_ID( server_ip ):
    return get_ip_address( server_ip ).split('.')[-1] - 1
