#!/usr/bin/env python
# -*- coding: utf8 -*-

import sys
import signal
import time
from multiprocessing import Queue
import paho.mqtt.client as mqtt
import paho.mqtt.publish as mqtt_publish
from a3dLogger import a3dLogger

# TODO:
#  - Exceptions

class MQTTHandler():

    def __init__(self, queue, host, port, log_level):
        self.queue = queue
        self.running = True
        self.host = host
        self.port = port

        self.logger = a3dLogger()
        self.logger.set_level(log_level)

        self.mqtt_client = mqtt.Client()

        self.mqtt_client.on_connect = self.__on_connect
        self.mqtt_client.on_disconnect = self.__on_disconnect
        self.mqtt_client.on_message = self.__on_message

    def __on_connect( self, client, userdata, flags, rc ):
        self.logger.info("Connected with result code %s" % str(rc))

    def __on_disconnect( self, client, userdata, rc ):
        self.logger.info("Disconnected with result code %s" % str(rc))

    def __on_message( self, client, userdata, msg ):
        self.logger.debug("Message received - Topic: `%s´ Msg: `%s´"
                        % (msg.topic, str(msg.payload)))
        # self.queue.put((msg.topic, msg.payload.decode()))
        self.queue.put((msg.topic, msg.payload))

    def start( self ):
        self.logger.info("Connecting to MQTT Broker.")
        self.mqtt_client.connect(self.host, self.port, 60)
        self.mqtt_client.loop_start()

    def stop( self ):
        self.logger.info("Disconnecting from MQTT Broker.")
        self.mqtt_client.loop_stop( force=False )
        self.mqtt_client.disconnect()

    def subscribe( self, topic, callback=None ):
        self.mqtt_client.subscribe(topic)
        if(callback is not None):
            self.mqtt_client.message_callback_add(topic, callback)

    def unsubscribe( self, topic ):
        self.mqtt_client.unsubscribe(topic)
        self.mqtt_client.message_callback_remove(topic)

    def publish_message( self, topic, msg ):
        mqtt_publish.single(topic, msg, hostname=self.host)

    def sub_matches_topic(self, sub, topic):
        return self.mqtt_client.topic_matches_sub(sub, topic)



def signal_handler( signal , frame ):
    global running
    running = False

def test_callback(mosq, obj, msg):
    print("EXIT: "+msg.topic+" "+str(msg.qos)+" "+str(msg.payload))             # TODO

if __name__ == '__main__':

    # add signal handler for SIGINT
    signal.signal( signal.SIGINT , signal_handler )

    # init stuff
    q = Queue()
    mqtt_client = MQTTHandler(q, "localhost", 1883, 7)
    mqtt_client.start()

    mqtt_client.subscribe("test/foo/1/bar")
    mqtt_client.subscribe("test/foo/2/bar", test_callback)

    mqtt_client.publish_message( "test/foo/1/bar", "Test no 1" )
    mqtt_client.publish_message( "test/foo/2/bar", "Test no 2" )

    # main program loop
    running = True
    while ( running ):
        if ( not q.empty() ):
            topic, msg = q.get()
            print( topic + " " + str(msg) )
        time.sleep( 0.1 )

    # cleanup
    mqtt_client.stop()
    sys.exit(0)
