#!/usr/bin/env python
# -*- coding: utf8 -*-

import sys
import signal
import time
import argparse
import threading
from multiprocessing import Queue
from envirophat import weather as envirophat_weather
from envirophat import light as envirophat_light

from a3dUtils import *
from a3dLogger import a3dLogger
from a3dSettings import SettingsHandler
from a3dMQTT import MQTTHandler
from a3dOSC import OSCHandler


# ============================================================================//
# ------------------------------------------------------------// Argument parser

#------------------------------------------/
#---/ Get arguments
parser = argparse.ArgumentParser()
parser.add_argument( "--DEBUG", action='store_true', help="Set debugging mode" )
args = parser.parse_args()
log_level = 3
if (args.DEBUG):
    log_level = 7

#------------------------------------------/
#---/ Set logger
logger = a3dLogger()
if (args.DEBUG):
    logger.set_level(log_level)
else:
    logger.set_level(log_level)

# ------------------------------------------------------------// Argument parser
# ----------------------------------------------------------------------------//



# ============================================================================//
# -----------------------------------------------------------// Global variables

version = "1.0.0"

server_ip = '192.168.1.1'                   # IP address of the Ubuntu server
broadcast_address = '192.168.255.255'       # Network broadcasting address

pi_id = get_ID( server_ip )                 # ID of the RPi - LSB of IP address

mqtt_broker = server_ip                     # IP address of MQTT Broker
mqtt_port = 1883                            # Port used for MQTT

# -----------------------------------------------------------// Global variables
# ----------------------------------------------------------------------------//



# ============================================================================//
# ------------------------------------------------------------// Local variables

running = True                              # Boolean used to indicate if app is running

# ------------------------------------------------------------// Local variables
# ----------------------------------------------------------------------------//



# ============================================================================//
# --------------------------------------------------------------------// Sensors

#------------------------------------------/
#---/ Sensor class
class Sensor():

    def __init__(self, id_, name, get_value):
        self.id = id_
        self.name = name
        self.value = 0
        self.timestamp = 0
        self.get_value = get_value
        self.on_interval = True
        self.interval = 5
        self.on_change = False
        self.change_threshold = 1
        self.broadcast_mqtt = True
        self.broadcast_osc = False

    def __str__(self):
        return ("%-12s inverval/change/MQTT/OSC - %s/%s/%s/%s/%s/%s"
                % (str(self.name),
                str(self.on_interval),
                str(self.interval),
                str(self.on_change),
                str(self.change_threshold),
                str(self.broadcast_mqtt),
                str(self.broadcast_osc)))

    def get_reading(self):
        self.value = self.get_value()
        self.timestamp = current_milli_time()

    def get_settings(self):
        return (str( int(self.id) ) +
                ',' + self.name +
                ',' + str(int(self.on_interval)) +
                ',' + str(self.interval) +
                ',' + str(int(self.on_change)) +
                ',' + str(self.change_threshold) +
                ',' + str(int(self.broadcast_mqtt)) +
                ',' + str(int(self.broadcast_osc)))

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
#---/ Define sensors list
sensors = [
    Sensor(0, "temperature", get_temperature),
    Sensor(1, "pressure", get_pressure),
    Sensor(2, "light", get_light),
    Sensor(3, "pedestrian", get_pedestrians)
    ]

#------------------------------------------/
#---/ check if value has changed
def check_changed( old_val, val, threshold ):
    return val > old_val + threshold or val < old_val - threshold

#------------------------------------------/
#---/ sensor loop
def sensor_loop():
    global running, mqtt_client, osc_handler, opencv
    global settings

    while ( running ):

        current_time = current_milli_time()

        for sensor in sensors:
            v = sensor.get_value()
            send = False

            if (sensor.on_interval and sensor.on_change):
                if (check_changed( sensor.value, v, sensor.change_threshold) or current_time >= sensor.timestamp ):
                    send = True
            elif (sensor.on_change):
                if (check_changed(sensor.value, v, senor.change_threshold)):
                    send = True
            elif (sensor.on_interval):
                if (current_time >= sensor.timestamp):
                    send = True

            if(send):
                if (sensor.broadcast_mqtt):
                    mqtt_client.publish_message(global_mqtt_topic + "/" + sensor.name, v)

                if (sensor.broadcast_osc):
                    osc_handler.send_message(global_osc_address + "/" + sensor.name, v)

                sensor.value = v
                sensor.timestamp = current_time + sensor.interval * 1000

# --------------------------------------------------------------------// Sensors
# ----------------------------------------------------------------------------//



# ============================================================================//
# -------------------------------------------------------------------// Settings

settings = SettingsHandler("settings")                          # Handler for settings

osc_port = settings.set("osc_port", 8008)                       # Port to broadcast OSC messages on
heartbeat_interval = settings.set("heartbeat_interval", 10)     # Seconds between heartbeats
sensors = settings.set("settings", sensors)                     # Sensor settings

#------------------------------------------/
#---/ Settings
def set_settings(mosq, obj, msg):
    global settings, heartbeat_message, osc_port, sensors

    s = str(msg.payload.decode()).split(',')

    # OSC port
    osc_port = settings.store("osc_port", int(s[0]))

    i = 1
    for sensor in sensors:
        sensor.on_interval = bool(int(s[i]))
        sensor.interval = int(s[i+1])
        sensor.on_change = bool(int(s[i+2]))
        sensor.change_threshold = float(s[i+3])
        sensor.broadcast_mqtt = bool(int(s[i+4]))
        sensor.broadcast_osc = bool(int(s[i+5]))
        i += 6

    sensors = settings.store("settings", sensors)

    build_heartbeat_message()
    send_heartbeat_message()

    logger.debug("Settings set:")
    logger.debug("OSC port                              - " + str(osc_port))
    for sensor in sensors:
        logger.debug(sensor)

# -------------------------------------------------------------------// Settings
# ----------------------------------------------------------------------------//



# ============================================================================//
# ------------------------------------------------------------------// Heartbeat

next_heartbeat = current_milli_time()

# -----------------------------------------------/
# ---/ Build the heartbeat message
def build_heartbeat_message():
    global heartbeat_message
    heartbeat_message = str( osc_port )
    for sensor in sensors:
        heartbeat_message += ',' + sensor.get_settings()

# -----------------------------------------------/
# ---/ Send the heartbeat message
def send_heartbeat_message():
    global mqtt_client, next_heartbeat, heartbeat_message
    next_heartbeat += heartbeat_interval * 1000
    mqtt_client.publish_message( heartbeat_topic, heartbeat_message )
    logger.debug( "Heartbeat message: " + heartbeat_message )
    mqtt_client.publish_message( version_topic, version )

#------------------------------------------/
#---/ send heartbeat
def hearbeat():
    global running
    build_heartbeat_message()
    while (running):
        if (current_milli_time() >= next_heartbeat):
            send_heartbeat_message()
        time.sleep( 0.1 )

# ------------------------------------------------------------------// Heartbeat
# ----------------------------------------------------------------------------//



# ============================================================================//
# -------------------------------------------------------------// Signal handler

# -----------------------------------------------/
# ---/ catch Ctrl+C to end program
def signal_handler(signal, frame):
    global running
    running = False

# -------------------------------------------------------------// Signal handler
# ----------------------------------------------------------------------------//



# ============================================================================//
# -----------------------------------------------------------------------// MQTT

# -----------------------------------------------/
# ---/ MQTT topics
global_mqtt_topic = 'parken/rpi/' + str(pi_id)
version_topic = global_mqtt_topic + '/version'
quit_topic = global_mqtt_topic + '/quit'
heartbeat_topic = global_mqtt_topic + '/heartbeat'
settings_topic = global_mqtt_topic + '/settings'

# -----------------------------------------------------------------------// MQTT
# ----------------------------------------------------------------------------//



# ============================================================================//
# ------------------------------------------------------------------------// OSC

# -----------------------------------------------/
# ---/ OSC addresses
global_osc_address = '/rpi/' + str(pi_id)

# ------------------------------------------------------------------------// OSC
# ----------------------------------------------------------------------------//



# ============================================================================//
# ---------------------------------------------------------// Main program logic

if __name__ == '__main__':

    logger.info("Raspberry Pi node with ID " + str(pi_id) + " starting...")

    # -----------------------------------------------/
    # ---/ Register signal handler for Ctrl+C
    signal.signal(signal.SIGINT, signal_handler)

    # -----------------------------------------------/
    # ---/ Initialise MQTT thread
    q = Queue()
    mqtt_client = MQTTHandler(q, mqtt_broker, mqtt_port, log_level)
    mqtt_client.start()
    mqtt_client.subscribe(quit_topic)
    mqtt_client.subscribe(settings_topic, set_settings)

    # -----------------------------------------------/
    # ---/ Initialise OSC handler
    osc_handler = OSCHandler(broadcast_address, osc_port, log_level)

    # -----------------------------------------------/
    # ---/ Initialise heartbeat thread
    heartbeat_thread = threading.Thread(target=hearbeat)
    heartbeat_thread.start()

    # -----------------------------------------------/
    # ---/ Initialise OpenCV thread
    opencv = OpenCVHandler( 640, 480, 32 )
    opencv.start()

    # -----------------------------------------------/
    # ---/ Initialise sensor thread
    sensor_thread = threading.Thread(target=sensor_loop)
    sensor_thread.start()


    # -----------------------------------------------/
    # ---/ Main program loop
    while (running):
        if (not q.empty()):
            topic, msg = q.get()
            logger.info("MQTT msg received: %s - %s" % (topic, msg.decode()))
            if(topic == quit_topic):           # TODO change to mqtt_client.sub_matches_topic(topic, settings_topic)
                running = False
        time.sleep( 0.1 )

    # -----------------------------------------------/
    # ---/ Cleanup
    logger.info('Closing program!')
    sys.exit( 0 )

# ---------------------------------------------------------// Main program logic
# ----------------------------------------------------------------------------//
