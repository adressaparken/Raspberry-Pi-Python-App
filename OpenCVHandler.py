#!/usr/bin/python

# Version:      0.1
# Last changed: 06/08/17

import sys
import signal
import time
import threading
import cv2
import imutils
import numpy
from imutils.object_detection import non_max_suppression
from picamera.array import PiRGBArray
from picamera import PiCamera
from my_logging import *

class OpenCVHandler(threading.Thread):

    def __init__( self, res_w, res_h, framerate ):
        threading.Thread.__init__(self)
        self.camera = PiCamera()
        self.camera.resolution = (res_w, res_h)
        self.camera.framerate = framerate
        self.rawCapture = PiRGBArray(self.camera, size=(res_w, res_h))

        # initialize the HOG descriptor/person detector
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

        self.num_detected = 0

    def background_subtraction( self, previous_frame, frame, min_area):
        frameDelta = cv2.absdiff( previous_frame, frame )
        thresh = cv2.threshold( frameDelta, 25, 255, cv2.THRESH_BINARY )[1]
        thresh = cv2.dilate( thresh, None, iterations=2 )
        im2, cnts, hierarchy = cv2.findContours( thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE )
        temp=0
        for c in cnts:
            if cv2.contourArea(c) > min_area:
                temp=1
        return temp

    def run( self ):
        self.running = True
        # camera warmup
        time.sleep(0.1)

        frame_num = 0

        for frame in self.camera.capture_continuous(self.rawCapture, format="bgr", use_video_port=True):
            start_time = current_milli_time()

            image = frame.array

            log_info("Grabbed image: " + str(start_time-current_milli_time()))

            # detect people in the image
            (rects, weights) = self.hog.detectMultiScale(image, winStride=(4, 4), padding=(8, 8), scale=1.05)

            log_info("Detect People: " + str(start_time-current_milli_time()))

            # apply non-maxima suppression to the bounding boxes using a
            # fairly large overlap threshold to try to maintain overlapping
            # boxes that are still people
            rects = numpy.array([[x, y, x + w, y + h] for (x, y, w, h) in rects])
            pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)

            log_info("Pick: " + str(start_time-current_milli_time()))

            # draw the final bounding boxes
            for (xA, yA, xB, yB) in pick:
                cv2.rectangle(image, (xA, yA), (xB, yB), (0, 255, 0), 2)

            # save number of people detected
            self.num_detected = len(pick)

            # show the output images
            # cv2.imshow("After NMS", image)
            # key = cv2.waitKey(1) & 0xFF

            # filename = "image" + str(frame_num) + ".jpg"
            # cv2.imwrite(filename,image)
            # cv2.imwrite('image_processes.jpg',image_processes)

            print( "People detected(" + str(frame_num) + "): " + str(self.num_detected) )

            # clear the stream in preparation for the next frame
            self.rawCapture.truncate(0)

            log_info("Finished: " + str(start_time-current_milli_time()))

            frame_num += 1

            if (not self.running):
                break


    def stop_thread( self ):
        self.running = False

    def get_num_detected( self ):
        return self.num_detected



def signal_handler( signal , frame ):
    global running
    running = False



if __name__ == '__main__':

    # add signal handler for SIGINT
    signal.signal( signal.SIGINT , signal_handler )

    # init stuff
    opencv = OpenCVHandler( 400, 300, 15 )
    opencv.start()

    # main program loop
    running = True
    while ( running ):
        print( opencv.get_num_detected() )
        time.sleep( 1 )

    # cleanup
    log_info( 'Closing program!' )
    opencv.stop_thread()
    opencv.join()
    sys.exit(0)
