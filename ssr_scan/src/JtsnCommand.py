#!/usr/bin/env python

#   MoveRob.py
#   
#   Author: Dean Taipale
#
#   Contributors: 
#   
#   Description:

import rospy
import sys
from std_msgs.msg import String
from threading import Semaphore



#readySem = Semaphore()

p = rospy.Publisher('jtsn_cmd', String, queue_size=10)
#rospy.init_node('publisher_node', anonymous=True)

#   
#   Function: run_command_publisher
#   Description: Run the command-sending node
#   

def run_command_publisher():
    rospy.loginfo("trying init_node")
    
    #rospy.init_node('publisher_node', anonymous=True)
    #p = rospy.Publisher('jtsn_cmd', String, queue_size=10)
    #rospy.init_node('publisher_node', anonymous=True)

    rospy.loginfo("Publisher running\n")

    r = rospy.Rate(1)

    while not rospy.is_shutdown():
        take_image()
    	#for line in sys.stdin:
        #    p.publish(line)
        r.sleep() 

    rospy.loginfo("Publisher shutdown")
