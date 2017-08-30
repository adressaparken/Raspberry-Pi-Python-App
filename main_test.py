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
# from OpenCVHandler import OpenCVHandler
# from envirophat import weather as envirophat_weather
# from envirophat import light as envirophat_light


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
temperature_interval = settings.set( "temperature_interval", 5 )
temperature_mqtt = settings.set( "temperature_mqtt", True )
temperature_osc = settings.set( "temperature_osc", True )

# Pressure settings
pressure_interval = settings.set( "pressure_interval", 5 )
pressure_mqtt = settings.set( "pressure_mqtt", True )
pressure_osc = settings.set( "pressure_osc", True )

# Pressure settings
light_interval = settings.set( "light_interval", 5 )
light_mqtt = settings.set( "light_mqtt", True )
light_osc = settings.set( "light_osc", True )

# Pedestrians settings
pedestrians_interval = settings.set( "pedestrians_interval", 5 )
pedestrians_mqtt = settings.set( "pedestrians_mqtt", True )
pedestrians_osc = settings.set( "pedestrians_osc", False )

# Sensor values
temperature = ( 0, current_milli_time() )
pressure = ( 0, current_milli_time() )
light = ( 0, current_milli_time() )
pedestrians = ( 0, current_milli_time() )
decibel = ( 0, current_milli_time() )

# Heartbeat_message - including all settings
heartbeat_message = str( osc_port )
heartbeat_message += ',' + str( temperature_interval ) + ',' + str( int(temperature_mqtt) ) + ',' + str( int(temperature_osc) )
heartbeat_message += ',' + str( pressure_interval ) + ',' + str( int(pressure_mqtt) ) + ',' + str( int(pressure_osc) )
heartbeat_message += ',' + str( light_interval ) + ',' + str( int(light_mqtt) ) + ',' + str( int(light_osc) )
heartbeat_message += ',' + str( pedestrians_interval ) + ',' + str( int(pedestrians_mqtt) ) + ',' + str( int(pedestrians_osc) )
# heartbeat_message += ',' + str( decibel_interval ) + ',' + str( int(decibel_mqtt) ) + ',' + str( int(decibel_osc) )

# MQTT topics
global_mqtt_topic = 'parken/rpi/' + str(pi_id)
quit_topic = global_mqtt_topic + '/quit'
heartbeat_topic = global_mqtt_topic + '/heartbeat'
settings_topic = global_mqtt_topic + '/settings'
temperature_mqtt_topic = global_mqtt_topic + '/temperature'
pressure_mqtt_topic = global_mqtt_topic + '/pressure'
light_mqtt_topic = global_mqtt_topic + '/light'
pedestrians_mqtt_topic = global_mqtt_topic + '/pedestrians'
decibel_mqtt_topic = global_mqtt_topic + '/decibel'

# OSC addressestopic + '/light'
global_osc_address = '/rpi/' + str(pi_id)
temperature_osc_address = global_osc_address + '/temperature'
pressure_osc_address = global_osc_address + '/pressure'
light_osc_address = global_osc_address + '/light'
pedestrians_osc_address = global_osc_address + '/pedestrians'
decibel_osc_address = global_osc_address + '/decibel'

##------------------------------------------------------------------------// Globals
##--------------------------------------------------------------------------------//



##================================================================================//
##-----------------------------------------------------------------------// Settings

#------------------------------------------/
#---/ Settings
def set_settings( s ):
    global heartbeat_message
    global osc_port
    global temperature_interval, temperature_mqtt, temperature_osc
    global pressure_interval, pressure_mqtt, pressure_osc
    global light_interval, light_mqtt, light_osc
    global pedestrians_interval, pedestrians_mqtt, pedestrians_osc
    global decibel_interval, decibel_mqtt, decibel_osc

    print( s )

    s = s.split(',')

    # OSC port
    osc_port = settings.store( "osc_port", int(s[0]) )

    # Temperature
    temperature_interval = settings.store( "temperature_interval", int(s[1]) )
    temperature_mqtt = settings.store( "temperature_mqtt", bool(int(s[2])) )
    temperature_osc = settings.store( "temperature_osc", bool(int(s[3])) )

    # Pressure
    pressure_interval = settings.store( "pressure_interval", int(s[4]) )
    pressure_mqtt = settings.store( "pressure_mqtt", bool(int(s[5])) )
    pressure_osc = settings.store( "pressure_osc", bool(int(s[6])) )

    # Light
    light_interval = settings.store( "light_interval", int(s[7]) )
    light_mqtt = settings.store( "light_mqtt", bool(int(s[8])) )
    light_osc = settings.store( "light_osc", bool(int(s[9])) )

    # Pedestrians
    pedestrians_interval = settings.store( "pedestrians_interval", int(s[10]) )
    pedestrians_mqtt = settings.store( "pedestrians_mqtt", bool(int(s[11])) )
    pedestrians_osc = settings.store( "pedestrians_osc", bool(int(s[12])) )

    # Pedestrians
    # decibel_interval = settings.store( "decibel_interval", int(s[13]) )
    # decibel_mqtt = settings.store( "decibel_mqtt", bool(int(s[14])) )
    # decibel_osc = settings.store( "decibel_osc", bool(int(s[15])) )

    # averaged values also (over a day?)
    # add sound stuff (amplitude, etc.)
    # direction of people walking? etc.
    # reload certain parts of web page (values, etc.)
    # only reload parts when
    # add text when pressing send that it's being updated...

    heartbeat_message = str( osc_port )
    heartbeat_message += ',' + str( temperature_interval ) + ',' + str( int( temperature_mqtt ) ) + ',' + str( int( temperature_osc ) )
    heartbeat_message += ',' + str( pressure_interval ) + ',' + str( int( pressure_mqtt ) ) + ',' + str( int( pressure_osc ) )
    heartbeat_message += ',' + str( light_interval ) + ',' + str( int( light_mqtt ) ) + ',' + str( int( light_osc ) )
    heartbeat_message += ',' + str( pedestrians_interval ) + ',' + str( int( pedestrians_mqtt ) ) + ',' + str( int( pedestrians_osc ) )
    # heartbeat_message += ',' + str( decibel_interval ) + ',' + str( int(decibel_mqtt) ) + ',' + str( int(decibel_osc) )

    mqtt_client.publish_message( heartbeat_topic, heartbeat_message )

    log_info( "Settings set: " + heartbeat_message )
    log_info( "OSC port                      - " + str( osc_port ) )
    log_info( "Temperature interval/MQTT/OSC - " + str( temperature_interval ) + "/" + str( temperature_mqtt ) + "/" + str( temperature_osc ) )
    log_info( "Pressure interval/MQTT/OSC    - " + str( pressure_interval ) + "/" + str( pressure_mqtt ) + "/" + str( pressure_osc ) )
    log_info( "Light interval/MQTT/OSC       - " + str( light_interval ) + "/" + str( light_mqtt ) + "/" + str( light_osc ) )
    log_info( "Pedestrians interval/MQTT/OSC - " + str( pedestrians_interval ) + "/" + str( pedestrians_mqtt ) + "/" + str( pedestrians_osc ) )
    # log_info( "Decibel interval/MQTT/OSC - " + str( decibel_interval ) + "/" + str( decibel_mqtt ) + "/" + str( decibel_osc ) )

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
##--------------------------------------------------------------------------// Sound

CHUNK = 2048
RATE = 48000

#------------------------------------------/
#---/ send decibel level
def send_decibel():
    global running, mqtt_client, osc_handler
    global decibel

    p=pyaudio.PyAudio()
    stream=p.open(format=pyaudio.paInt16,channels=1,rate=RATE,input=True,
                  frames_per_buffer=CHUNK)

    while ( running ):

        current_time = current_milli_time()

        for i in range(int(10*44100/1024)): #go for a few seconds
            data = np.fromstring(stream.read(CHUNK),dtype=np.int16)
            rms = audioop.rms(data, 2)
            d = 20 * math.log10(rms)

            if ( decibel_mqtt ):
                mqtt_client.publish_message( decibel_mqtt_topic, d )

            if ( decibel_osc ):
                osc_handler.send_message( decibel_osc_address, str( d ) )

            decibal = (d, current_time + decibel_interval * 1000)

            if( not running ):
                break

        time.sleep( 0.1 )

    stream.stop_stream()
    stream.close()
    p.terminate()

##--------------------------------------------------------------------------// Sound
##--------------------------------------------------------------------------------//



##================================================================================//
##--------------------------------------------------------------------// Sensor data

#------------------------------------------/
#---/ get temperature
def get_temperature():
    # return round( envirophat_weather.temperature(), 1)
    return 1

#------------------------------------------/
#---/ get pressure
def get_pressure():
    # return round( envirophat_weather.pressure(), 1)
    return 1

#------------------------------------------/
#---/ get pressure
def get_light():
    # return round( envirophat_light.light(), 1)
    return 1

#------------------------------------------/
#---/ get pressure
def get_pedestrians():
    # return opencv.get_num_detected()
    return 1

#------------------------------------------/
#---/ sensor loop
def sensor_loop():
    global running, mqtt_client, osc_handler, opencv
    global temperature, pressure, light, pedestrians

    while ( running ):

        current_time = current_milli_time()

        # get temperature
        t = get_temperature()
        # if temperature changed or timer has passed, send
        if ( current_time >= temperature[1] ):
            if ( temperature_mqtt ):
                mqtt_client.publish_message( temperature_mqtt_topic, t )

            if ( temperature_osc ):
                osc_handler.send_message( temperature_osc_address, str( t ) )

            temperature = (t, current_time + temperature_interval * 1000)

        # get pressure
        p = get_pressure()
        # if pressure changed or timer has passed, send
        if ( current_time >= pressure[1] ):
            if ( pressure_mqtt ):
                mqtt_client.publish_message( pressure_mqtt_topic, p )

            if ( pressure_osc ):
                osc_handler.send_message( pressure_osc_address, str( p ) )

            pressure = (p, current_time + pressure_interval * 1000)

        # get light
        l = get_light()
        # if light changed or timer has passed, send
        if ( current_time >= light[1] ):
            if ( light_mqtt ):
                mqtt_client.publish_message( light_mqtt_topic, l )

            if ( light_osc ):
                osc_handler.send_message( light_osc_address, str( l ) )

            light = (l, current_time + light_interval * 1000)

        # get pedestrians
        pe = get_pedestrians()
        # if pedestrians changed or timer has passed, send
        if ( pe is not pedestrians[0] or current_time >= pedestrians[1] ):
            if ( pedestrians_mqtt ):
                mqtt_client.publish_message( pedestrians_mqtt_topic, pe )

            if ( pedestrians_osc ):
                osc_handler.send_message( pedestrians_osc_address, str( pe ) )

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
    global running, pi_id, mqtt_client, osc_handler#, opencv
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
    # opencv = OpenCVHandler( 640, 480, 32 )
    # opencv.start()

    # Initialise sensor thread
<<<<<<< HEAD
    # decibel_thread = threading.Thread( target=send_decibel )
    # decibel_thread.start()
=======
    #decibel_thread = threading.Thread( target=send_decibel )
    #decibel_thread.start()
>>>>>>> 1f28f96a7603a6b6edb551379039aa1acca30243

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
    # opencv.stop_thread()
    # opencv.join()
    heartbeat_thread.join()
    # decibel_thread.join()
    sensor_thread.join()
<<<<<<< HEAD
=======
    #decibel_thread.join()
>>>>>>> 1f28f96a7603a6b6edb551379039aa1acca30243
    mqtt_client.stop()
    log_info( 'Closing program!' )
    sys.exit( 0 )


if __name__ == '__main__':
    main()

##-------------------------------------------------------------// Main program logic
##--------------------------------------------------------------------------------//
