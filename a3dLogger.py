#!/usr/bin/python

# Version:      0.1
# Last changed: 01/01/17

import sys
import signal
import time

'''
## TODO:
##  - file
##  - formatter
##  - log_levels: TRACE, DEBUG, INFO, WARN, ERROR, FATAL
##    https://stackoverflow.com/questions/2031163/when-to-use-the-different-log-levels
'''


# ============================================================================//
# ------------------------------------------------------------------// a3dLogger

class a3dLogger():

    '''
    ## Log level:
    ## possibilities:
    ## 0: no logging (default)
    ## 1: log INFO
    ## 2: log ERROR
    ## 3: log INFO and ERROR
    ## 4: log DEBUG
    ## 5: log INFO and DEBUG
    ## 6: log ERROR and DEBUG
    ## 7: log INFO, ERROR and DEBUG
    '''

    NONE = 0x00
    INFO = 0x01
    ERROR = 0x02
    DEBUG = 0x04

    def __init__(self):
        self.log_level = a3dLogger.NONE

    def set_level(self, log_level):
        self.log_level = log_level

    def __log(self, log_level, msg):
        print("<%-5s %s-%s> %s" % (log_level,
                                   time.strftime("%d/%m/%y"),
                                   time.strftime("%H:%M:%S"),
                                   msg))

    def info(self, msg):
        if(self.log_level & a3dLogger.INFO):
            self.__log("INFO", msg)

    def error(self, msg):
        if(self.log_level & a3dLogger.ERROR):
            self.__log("ERROR", msg)

    def debug(self, msg):
        if(self.log_level & a3dLogger.DEBUG):
            self.__log("DEBUG", msg)

# ------------------------------------------------------------------// a3dLogger
# ----------------------------------------------------------------------------//



# ============================================================================//
# -----------------------------------------------------------------// Exceptions

class Error(Exception):
    """Base class for exceptions in this module."""
    pass


class InputError(Error):
    """Exception raised for errors in the input.

    Attributes:
        expression -- input expression in which the error occurred
        message -- explanation of the error
    """

    def __init__(self, expression, message):
        self.expression = expression
        self.message = message

# -----------------------------------------------------------------// Exceptions
# ----------------------------------------------------------------------------//



# ============================================================================//
# ---------------------------------------------------------// Main program logic

if __name__ == '__main__':
    logger = a3dLogger()
    logger.set_level(7);
    logger.info("Info logging")
    logger.error("Error logging")
    logger.debug("Debug logging")

# ---------------------------------------------------------// Main program logic
# ----------------------------------------------------------------------------//
