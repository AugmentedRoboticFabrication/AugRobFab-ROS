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

subprocess.Popen('rosrun image_view image_saver image:=/rgb/image_raw _save_all_image:=false __name:=image_saver'.split())


def take_image():
    rospy.sleep(delay)
    os.system('rosservice call /image_saver/save')
    
    
# Debug methods

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
