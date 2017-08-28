#!/usr/bin/python

# Version:      0.1
# Last changed: 01/01/17

import sys
import time

DEBUG = False

##================================================================================//
##------------------------------------------------------------------------// Logging

def set_debug( d ):
    global DEBUG
    DEBUG = d

#------------------------------------------/
#---/ Log message - INFO
def log( log_level, msg ):
    print( "<%-5s %s-%s> %s" % (log_level, time.strftime("%d/%m/%y"), time.strftime("%H:%M:%S"), msg) )

#------------------------------------------/
#---/ Log message - INFO
def log_info( msg ):
    log( "INFO", msg )

#------------------------------------------/
#---/ Log message - ERROR
def log_error( msg ):
    log( "ERROR", msg )

#------------------------------------------/
#---/ Log message - DEBUG
def log_debug( msg ):
    if( DEBUG ):
        log( "DEBUG", msg )

##------------------------------------------------------------------------// Logging
##--------------------------------------------------------------------------------//
