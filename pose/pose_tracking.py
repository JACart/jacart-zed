#!/usr/bin/env python
import roslib
import rospy
import message_filters
from message_filters import TimeSynchronizer
import sys
import os
import math
import time
import json
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image
from zed_interfaces.msg import Object, ObjectsStamped
import numpy as np
# import cv2
# from cv_bridge import CvBridge, CvBridgeError

MAX_RATE = 5.0

class pose_tracking:

    def __init__(self):
        rospy.init_node('pose_tracker')
        rospy.loginfo("Started pose tracking node!")

        # subscribing to depth image and color version
        img_sub = message_filters.Subscriber("/passenger_cam/passenger_zed/left/image_rect_color", Image)
        depth_sub = message_filters.Subscriber("/passenger_cam/passenger_zed/depth/depth_registered", Image)
        self.obj_sub = message_filters.Subscriber("/passenger_cam/passenger_zed/obj_det/objects", ObjectsStamped)

        self.passenger_safe_pub = rospy.Publisher('/passenger_safe', Bool, queue_size=10)
        self.passenger_exit_pub = rospy.Publisher('/passenger_exit', Bool, queue_size=10)
        rospy.Subscriber('/safety_constant', Bool, self.initial_safety)
        rospy.Subscriber('/safety_exit', Bool, self.passenger_exit)

        self.empty = True

        self.objects = None

        self.trip_live = False

        self.last_time = -1

        self.obj_sub.registerCallback(self.obj_sub_callback)

        self.img_depth_synch = TimeSynchronizer([img_sub, depth_sub],
                                                           queue_size=10)

        self.img_depth_synch.registerCallback(self.image_depth_callback)
        
        # publish cart empty
        self.cart_empty_safe_pub = rospy.Publisher('/cart_empty_safe', String, queue_size=10)

        self.image_pub = rospy.Publisher('pose_image', Image,
                                         queue_size=10)

        rospy.spin()
    
    def sendPassengerUnsafe(self):
        self.passenger_safe_pub.publish(False)


    def sendPassengerSafe(self):
        self.passenger_safe_pub.publish(True)


    def sendPassengerExit(self):
        self.passenger_exit_pub.publish(True)


    def safety_analysis(self):
        confidence = 0
        self.trip_live = True
        while objects is not None and self.trip_live and not rospy.is_shutdown():
            
            # Check passenger safety and update confidence level
            if (len(objects.objects) and confidence < (self.CONFIDENCE_THRESHOLD*2)):
                confidence += 1
            else:
                if confidence > 0:
                    confidence -= 1
                else:
                    confidence = 0

            # Check confidence against threshold and call unsafe method if necessary.
            if confidence >= self.CONFIDENCE_THRESHOLD:
                # Send unsafe message
                if self.passenger_unsafe == False:
                    self.sendPassengerUnsafe()
                    self.passenger_unsafe = True
            else:
                if self.passenger_unsafe == True:
                    self.passenger_unsafe = False
                    self.sendPassengerSafe()

    def initial_safety(self, data):
        safety_counter = 0
        # Send unsafe message
        if self.passenger_unsafe == False:
            self.sendPassengerUnsafe()
            self.passenger_unsafe = True
        
        if objects is not None:
            while safety_counter < 30 and not rospy.is_shutdown():
                if len(objects.objects) > 0:
                    safety_counter += 1
                else:
                    if safety_counter > 0:
                        safety_counter -= 1
                    else:
                        safety_counter = 0

            self.sendPassengerSafe()
            self.passenger_unsafe = False
            self.safety_analysis()            
            print("Not None")
    
    def passenger_exit(self, data):
        self.trip_live = False
        if objects is not None:
            while len(objects.objects) is not 0:
                time.sleep(1)
                

            rospy.loginfo("Sleeping while waiting for exit")
            time.sleep(5)  
            rospy.loginfo("Done Sleeping Sending Exit")
            self.sendPassengerExit()

    def sendCartEmptyPose(self, empty, safe):
        '''
        Ros topic passenger is present and passenger is safe
        '''
        empty_safe_state = json.dumps({'passenger': not empty, 'safe': safe})
        print(empty_safe_state)
        self.cart_empty_safe_pub.publish(empty_safe_state)
    
    def obj_sub_callback(self, objects):

        self.objects = objects
        # print(len(objects.objects))
        if len(objects.objects) > 0:
            self.sendCartEmptyPose(False, True)
        else:
            self.sendCartEmptyPose(True, False)

    def image_depth_callback(self, img_msg, depth_msg):
        '''
        Callback for left color zed image runs open pose publishes to empty safe
        '''
        cur_time = time.time()
        if cur_time < self.last_time + 1.0 / MAX_RATE:
            return
        self.last_time = cur_time

        self.image_pub.publish(img_msg)

if __name__ == "__main__":
    try:
        pose_tracking()
    except rospy.ROSInterruptException:
        pass
