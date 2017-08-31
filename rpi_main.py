#!/usr/bin/python

import sys
import signal
import time
import argparse
import threading
import socket
from multiprocessing import Queue
from utils import *
from my_logging import *
import pyaudio
import numpy as np
import audioop
import math
from SettingsHandler import SettingsHandler
from MQTTHandler import MQTTHandler
from OSCHandler import OSCHandler
from OpenCVHandler import OpenCVHandler
from envirophat import weather as envirophat_weather
from envirophat import light as envirophat_light


##================================================================================//
##------------------------------------------------------------------------// Globals

# Set debug level
parser = argparse.ArgumentParser()
parser.add_argument( "--DEBUG", action='store_true', help="Set debugging mode" )
args = parser.parse_args()
set_debug( args.DEBUG )

settings = SettingsHandler( "settings" )

server_ip = '192.168.1.1'
broadcast_address = '192.168.255.255'
pi_id = get_ID( server_ip )

running = True

# MQTT server address and port
mqtt_broker = server_ip
mqtt_port = 1883

# OSC port
osc_port = settings.set( "osc_port", 5005 )

# Heartbeat interval - seconds between heartbeats
heartbeat_interval = settings.set( "heartbeat_interval", 10 )

# Temperature settings
temperature_on_interval = settings.set( "temperature_on_interval", True )
temperature_interval = settings.set( "temperature_interval", 5 )
temperature_change = settings.set( "temperature_change", False )
temperature_change_threshold = settings.set( "temperature_change_threshold", 1.0 )
temperature_mqtt = settings.set( "temperature_mqtt", True )
temperature_osc = settings.set( "temperature_osc", True )

# Pressure settings
pressure_on_interval = settings.set( "pressure_on_interval", True )
pressure_interval = settings.set( "pressure_interval", 5 )
pressure_change = settings.set( "pressure_change", False )
pressure_change_threshold = settings.set( "pressure_change_threshold", 1.0 )
pressure_mqtt = settings.set( "pressure_mqtt", True )
pressure_osc = settings.set( "pressure_osc", True )

# Pressure settings
light_on_interval = settings.set( "light_on_interval", True )
light_interval = settings.set( "light_interval", 5 )
light_change = settings.set( "light_change", False )
light_change_threshold = settings.set( "light_change_threshold", 1 )
light_mqtt = settings.set( "light_mqtt", True )
light_osc = settings.set( "light_osc", True )

# Pedestrians settings
pedestrians_on_interval = settings.set( "pedestrians_on_interval", True )
pedestrians_interval = settings.set( "pedestrians_interval", 5 )
pedestrians_change = settings.set( "pedestrians_change", False )
pedestrians_change_threshold = settings.set( "pedestrians_change_threshold", 1 )
pedestrians_mqtt = settings.set( "pedestrians_mqtt", True )
pedestrians_osc = settings.set( "pedestrians_osc", False )

# Sensor values
temperature = ( 0, current_milli_time() )
pressure = ( 0, current_milli_time() )
light = ( 0, current_milli_time() )
pedestrians = ( 0, current_milli_time() )

# Heartbeat_message - including all settings
heartbeat_message = str( osc_port )
heartbeat_message += ',' + str( int(temperature_on_interval) ) + ',' + str( temperature_interval ) + ',' + str( int(temperature_change) ) + ',' + str( int(temperature_change_threshold) ) + ',' + str( int(temperature_mqtt) ) + ',' + str( int(temperature_osc) )
heartbeat_message += ',' + str( int(pressure_on_interval) ) + ',' + str( pressure_interval ) + ',' + str( int(pressure_change) ) + ',' + str( int(pressure_change_threshold) ) + ',' + str( int(pressure_mqtt) ) + ',' + str( int(pressure_osc) )
heartbeat_message += ',' + str( int(light_on_interval) ) + ',' + str( light_interval ) + ',' + str( int(light_change) ) + ',' + str( int(light_change_threshold) ) + ',' + str( int(light_mqtt) ) + ',' + str( int(light_osc) )
heartbeat_message += ',' + str( int(pedestrians_on_interval) ) + ',' + str( pedestrians_interval ) + ',' + str( int(pedestrians_change) ) + ',' + str( int(pedestrians_change_threshold) ) + ',' + str( int(pedestrians_mqtt) ) + ',' + str( int(pedestrians_osc) )

# MQTT topics
global_mqtt_topic = 'parken/rpi/' + str(pi_id)
quit_topic = global_mqtt_topic + '/quit'
heartbeat_topic = global_mqtt_topic + '/heartbeat'
settings_topic = global_mqtt_topic + '/settings'
temperature_mqtt_topic = global_mqtt_topic + '/temperature'
pressure_mqtt_topic = global_mqtt_topic + '/pressure'
light_mqtt_topic = global_mqtt_topic + '/light'
pedestrians_mqtt_topic = global_mqtt_topic + '/pedestrians'

# OSC addressestopic + '/light'
global_osc_address = '/rpi/' + str(pi_id)
temperature_osc_address = global_osc_address + '/temperature'
pressure_osc_address = global_osc_address + '/pressure'
light_osc_address = global_osc_address + '/light'
pedestrians_osc_address = global_osc_address + '/pedestrians'

##------------------------------------------------------------------------// Globals
##--------------------------------------------------------------------------------//



##================================================================================//
##-----------------------------------------------------------------------// Settings

#------------------------------------------/
#---/ Settings
def set_settings( s ):
    global heartbeat_message
    global osc_port
    global temperature_on_interval, temperature_interval, temperature_change, temperature_change_threshold, temperature_mqtt, temperature_osc
    global pressure_on_interval, pressure_interval, pressure_change, pressure_change_threshold, pressure_mqtt, pressure_osc
    global light_on_interval, light_interval, light_change, light_change_threshold, light_mqtt, light_osc
    global pedestrians_on_interval, pedestrians_interval, pedestrians_change, pedestrians_change_threshold, pedestrians_mqtt, pedestrians_osc

    s = s.split(',')

    # OSC port
    osc_port = settings.store( "osc_port", int(s[0]) )

    # Temperature
    temperature_on_interval = settings.store( "temperature_on_interval", bool(int(s[1])) )
    temperature_interval = settings.store( "temperature_interval", int(s[2]) )
    temperature_change = settings.store( "temperature_change", bool(int(s[3])) )
    temperature_change_threshold = settings.store( "temperature_change_threshold", float(s[4]) )
    temperature_mqtt = settings.store( "temperature_mqtt", bool(int(s[5])) )
    temperature_osc = settings.store( "temperature_osc", bool(int(s[6])) )

    # Pressure
    pressure_on_interval = settings.store( "pressure_on_interval", bool(int(s[7])) )
    pressure_interval = settings.store( "pressure_interval", int(s[8]) )
    pressure_change = settings.store( "pressure_change", bool(int(s[9])) )
    pressure_change_threshold = settings.store( "pressure_change_threshold", float(s[10]) )
    pressure_mqtt = settings.store( "pressure_mqtt", bool(int(s[11])) )
    pressure_osc = settings.store( "pressure_osc", bool(int(s[12])) )

    # Light
    light_on_interval = settings.store( "light_on_interval", bool(int(s[13])) )
    light_interval = settings.store( "light_interval", int(s[14]) )
    light_change = settings.store( "light_change", bool(int(s[15])) )
    light_change_threshold = settings.store( "light_change_threshold", float(s[16]) )
    light_mqtt = settings.store( "light_mqtt", bool(int(s[17])) )
    light_osc = settings.store( "light_osc", bool(int(s[18])) )

    # Pedestrians
    pedestrians_on_interval = settings.store( "pedestrians_on_interval", bool(int(s[19])) )
    pedestrians_interval = settings.store( "pedestrians_interval", int(s[20]) )
    pedestrians_change = settings.store( "pedestrians_change", bool(int(s[21])) )
    pedestrians_change_threshold = settings.store( "pedestrians_change_threshold", float(s[22]) )
    pedestrians_mqtt = settings.store( "pedestrians_mqtt", bool(int(s[23])) )
    pedestrians_osc = settings.store( "pedestrians_osc", bool(int(s[24])) )

    # averaged values also (over a day?)
    # add sound stuff (amplitude, etc.)
    # direction of people walking? etc.
    # reload certain parts of web page (values, etc.)
    # only reload parts when
    # add text when pressing send that it's being updated...

    heartbeat_message = str( osc_port )
    heartbeat_message += ',' + str( int(temperature_on_interval) ) + ',' + str( temperature_interval ) + ',' + str( int(temperature_change) ) + ',' + str( int(temperature_change_threshold) ) + ',' + str( int(temperature_mqtt) ) + ',' + str( int(temperature_osc) )
    heartbeat_message += ',' + str( int(pressure_on_interval) ) + ',' + str( pressure_interval ) + ',' + str( int(pressure_change) ) + ',' + str( int(pressure_change_threshold) ) + ',' + str( int(pressure_mqtt) ) + ',' + str( int(pressure_osc) )
    heartbeat_message += ',' + str( int(light_on_interval) ) + ',' + str( light_interval ) + ',' + str( int(light_change) ) + ',' + str( int(light_change_threshold) ) + ',' + str( int(light_mqtt) ) + ',' + str( int(light_osc) )
    heartbeat_message += ',' + str( int(pedestrians_on_interval) ) + ',' + str( pedestrians_interval ) + ',' + str( int(pedestrians_change) ) + ',' + str( int(pedestrians_change_threshold) ) + ',' + str( int(pedestrians_mqtt) ) + ',' + str( int(pedestrians_osc) )

    mqtt_client.publish_message( heartbeat_topic, heartbeat_message )

    log_info( "Settings set: " + heartbeat_message )
    log_info( "OSC port                      - " + str( osc_port ) )
    log_info( "Temperature interval/change/threshold/MQTT/OSC - " + str( temperature_on_interval ) + "/" + str( temperature_interval ) + "/" + str( temperature_change ) + "/" + str( temperature_change_threshold ) + "/" + str( temperature_mqtt ) + "/" + str( temperature_osc ) )
    log_info( "Pressure interval/change/threshold/MQTT/OSC    - " + str( pressure_on_interval ) + "/" + str( pressure_interval ) + "/" + str( pressure_change ) + "/" + str( pressure_change_threshold ) + "/" + str( pressure_mqtt ) + "/" + str( pressure_osc ) )
    log_info( "Light interval/change/threshold/MQTT/OSC       - " + str( light_on_interval ) + "/" + str( light_interval ) + "/" + str( light_change ) + "/" + str( light_change_threshold ) + "/" + str( light_mqtt ) + "/" + str( light_osc ) )
    log_info( "Pedestrians interval/change/threshold/MQTT/OSC - " + str( pedestrians_on_interval ) + "/" + str( pedestrians_interval ) + "/" + str( pedestrians_change ) + "/" + str( pedestrians_change_threshold ) + "/" + str( pedestrians_mqtt ) + "/" + str( pedestrians_osc ) )

##-----------------------------------------------------------------------// Settings
##--------------------------------------------------------------------------------//



##================================================================================//
##----------------------------------------------------------------------// Heartbeat

#------------------------------------------/
#---/ send heartbeat
def send_hearbeat():
    global running, mqtt_client
    next_frame = current_milli_time()
    while (running):
        if (current_milli_time() >= next_frame):
            next_frame += heartbeat_interval * 1000
            mqtt_client.publish_message( heartbeat_topic, heartbeat_message )
            log_debug( "Heartbeat message: " + heartbeat_message )
        time.sleep( 0.1 )

##----------------------------------------------------------------------// Heartbeat
##--------------------------------------------------------------------------------//



##================================================================================//
##--------------------------------------------------------------------// Sensor data

#------------------------------------------/
#---/ get temperature
def get_temperature():
    return round( envirophat_weather.temperature(), 1)

#------------------------------------------/
#---/ get pressure
def get_pressure():
    return round( envirophat_weather.pressure(), 1)

#------------------------------------------/
#---/ get pressure
def get_light():
    return round( envirophat_light.light(), 1)

#------------------------------------------/
#---/ get pressure
def get_pedestrians():
    return opencv.get_num_detected()

#------------------------------------------/
#---/ check if value has changed
def check_changed( old_val, val, threshold ):
    return val > old_val + threshold or val < old_val - threshold

#------------------------------------------/
#---/ sensor loop
def sensor_loop():
    global running, mqtt_client, osc_handler, opencv
    global temperature, pressure, light, pedestrians

    while ( running ):

        current_time = current_milli_time()

        # get temperature
        t = get_temperature()
        send = False
        # if temperature changed or timer has passed, send
        if ( temperature_on_interval and temperature_change ):
            if ( check_changed( temperature[0], t, temperature_change_threshold ) or current_time >= temperature[1] ):
                send = True
        elif( temperature_change ):
            if( check_changed( temperature[0], t, temperature_change_threshold ) ):
                send = True
        elif( temperature_on_interval ):
            if ( current_time >= temperature[1] ):
                send = True

        if( send ):
            if ( temperature_mqtt ):
                mqtt_client.publish_message( temperature_mqtt_topic, t )

            if ( temperature_osc ):
                osc_handler.send_message( temperature_osc_address, t )

            temperature = (t, current_time + temperature_interval * 1000)

        # get pressure
        p = get_pressure()
        send = False
        # if pressure changed or timer has passed, send
        if ( pressure_on_interval and pressure_change ):
            if ( check_changed( pressure[0], p, pressure_change_threshold ) or current_time >= pressure[1] ):
                send = True
        elif( pressure_change ):
            if( check_changed( pressure[0], p, pressure_change_threshold ) ):
                send = True
        elif( pressure_on_interval ):
            if ( current_time >= pressure[1] ):
                send = True

        if( send ):
            if ( pressure_mqtt ):
                mqtt_client.publish_message( pressure_mqtt_topic, p )

            if ( pressure_osc ):
                osc_handler.send_message( pressure_osc_address, p )

            pressure = (p, current_time + pressure_interval * 1000)

        # get light
        l = get_light()
        send = False
        # if light changed or timer has passed, send
        if ( light_on_interval and light_change ):
            if ( check_changed( light[0], l, light_change_threshold ) or current_time >= light[1] ):
                send = True
        elif( light_change ):
            if( check_changed( light[0], l, light_change_threshold ) ):
                send = True
        elif( light_on_interval ):
            if ( current_time >= light[1] ):
                send = True

        if( send ):
            if ( light_mqtt ):
                mqtt_client.publish_message( light_mqtt_topic, l )

            if ( light_osc ):
                osc_handler.send_message( light_osc_address, l )

            light = (l, current_time + light_interval * 1000)

        # get pedestrians
        pe = get_pedestrians()
        send = False
        # if pedestrians changed or timer has passed, send
        if ( pedestrians_on_interval and pedestrians_change ):
            if ( check_changed( pedestrians[0], pe, pedestrians_change_threshold ) or current_time >= pedestrians[1] ):
                send = True
        elif( pedestrians_change ):
            if( check_changed( pedestrians[0], pe, pedestrians_change_threshold ) ):
                send = True
        elif( pedestrians_on_interval ):
            if ( current_time >= pedestrians[1] ):
                send = True

        if( send ):
            if ( pedestrians_mqtt ):
                mqtt_client.publish_message( pedestrians_mqtt_topic, pe )

            if ( pedestrians_osc ):
                osc_handler.send_message( pedestrians_osc_address, pe )

            pedestrians = (pe, current_time + pedestrians_interval * 1000)

        time.sleep( 0.1 )

##--------------------------------------------------------------------// Sensor data
##--------------------------------------------------------------------------------//



##================================================================================//
##----------------------------------------------------------------// Signal handlers

#------------------------------------------/
#---/ catch Ctrl + C to stop program
def signal_handler( signal , frame ):
    global running
    running = False

##----------------------------------------------------------------// Signal handlers
##--------------------------------------------------------------------------------//



##================================================================================//
##-------------------------------------------------------------// Main program logic

def main():
    global running, pi_id, mqtt_client, osc_handler, opencv
    global temperature, pressure, light, pedestrians

    log_info( "Raspberry Pi node with ID " + str( pi_id ) + " starting..." )

    # Add signal handler for SIGINT
    signal.signal( signal.SIGINT , signal_handler )

    # Initialise MQTT thread
    q = Queue()
    mqtt_client = MQTTHandler( q, mqtt_broker, mqtt_port )
    mqtt_client.start()
    mqtt_client.subscribe( quit_topic )
    mqtt_client.subscribe( settings_topic )

    # Initialise OSC handler
    osc_handler = OSCHandler( broadcast_address, osc_port )

    # Initialise heartbeat thread
    heartbeat_thread = threading.Thread( target=send_hearbeat )
    heartbeat_thread.start()

    # Initialise OpenCV thread
    opencv = OpenCVHandler( 640, 480, 32 )
    opencv.start()

    # Initialise sensor thread
    sensor_thread = threading.Thread( target=sensor_loop )
    sensor_thread.start()

    # Main program loop
    running = True
    while ( running ):
        if ( not q.empty() ):
            topic, msg = q.get()
            log_info( "MQTT msg received: %s - %s" % ( topic, msg.decode() ) )

            # Settings
            if (topic == settings_topic):
                set_settings( msg.decode() )
            elif (topic == quit_topic):
                running = False

        time.sleep( 0.1 )

    # Cleanup
    opencv.stop_thread()
    opencv.join()
    heartbeat_thread.join()
    sensor_thread.join()
    mqtt_client.stop()
    log_info( 'Closing program!' )
    sys.exit( 0 )


if __name__ == '__main__':
    main()

##-------------------------------------------------------------// Main program logic
##--------------------------------------------------------------------------------//
