#!/usr/bin/env python3

#   MoveRob.py
#   
#   Author: Dean Taipale
#
#   Contributors: Sebastian Schweitzer, Lin Jia
#   
#   Description: Methods for the PC side of PC-Jetson communication

#import yaml
import rospy
import sys
from std_msgs.msg import String
#from threading import Semaphore
import os
import subprocess

delay = 3

#runs a subprocess node to save our images when triggered
subprocess.Popen('rosrun image_view image_saver image:=/rgb/image_raw _save_all_image:=false __name:=image_saver'.split())


#   
#   Function: take_image
#
#   Description: Call the service to save an image from the Jetson's image topic stream
#   

def take_image():
    rospy.sleep(delay)
    
    #trigger saving
    os.system('rosservice call /image_saver/save')
    
    
# Debug methods

#   
#   Function: debug_client
#
#   Description: Interactible method run only if this script is run as main. Allows the user to manually trigger image-saving.
#   

def debug_client():
    rospy.init_node('debug_pc_node', anonymous=True)

    print("Command Debugger Node Running\nPlease run the jetson node now.")

    s = input("Is it running?[y/n]\n")

    if s != 'y':
        print("quiting")
        return

    while True:
        s = input("trigger image? [y/n]\n")

        if s != 'y':
            print("quiting")
            return
        
        take_image()



if __name__ == '__main__':
    try:
        
        debug_client()

    except rospy.ROSInterruptException:
        pass
