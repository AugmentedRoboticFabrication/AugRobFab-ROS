#!/usr/bin/env python

#   MoveRob.py
#   
#   Author: Dean Taipale
#
#   Contributors: Sebastian Schweitzer
#   
#   Description: Methods for the PC side of PC-Jetson communication

import rospy
import sys
from std_msgs.msg import String
from threading import Semaphore


p = rospy.Publisher('jtsn_cmd', String, queue_size=10)

#   
#   Function: feedback_callback
#   Description: called whenever the subscriber recieves data on jtsn_fbk
#  

def feedback_callback(data):
    print("Recieved feedback: " + str(data.data))  

rospy.Subscriber('jtsn_fbk', String, feedback_callback)

#   
#   Function: take_image
#   Description: send the command to take a picture
#  

def take_image():
    print("Sending Capture command")
    p.publish("x")

#   
#   Function: say
#   Description: send an arbitrary message to the jtsn_cmd channel
#   

def say(message):
    p.publish(message)
