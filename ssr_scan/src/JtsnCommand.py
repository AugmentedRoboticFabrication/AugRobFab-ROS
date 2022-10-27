#!/usr/bin/env python
import rospy
import sys
from std_msgs.msg import String

p = None

#   
#   Function: run_command_publisher
#   Description: Run the command-sending node
#   

def run_command_publisher():
    rospy.loginfo("trying init_node")
    
    p = rospy.Publisher('jtsn_cmd', String, queue_size=10)
    rospy.init_node('publisher_node', anonymous=True)

    rospy.loginfo("Publisher running\n")

    #r = rospy.Rate(1)
    #while not rospy.is_shutdown():
    #    r.sleep() 
    #rospy.loginfo("Publisher shutdown")

#   
#   Function: run_feedback_listener
#   Description: Run the feedback-listening node
#   

def run_feedback_listener():
    rospy.loginfo("trying init_node")

    rospy.init_node("subscriber_Node", anonymous=True)
    rospy.Subscriber('jtsn_feedback', String, callback)

    rospy.loginfo("Subscriber running\n")
    #rospy.spin()



def callback(data):
    print(data.data)
        
#   
#   Function: 
#   Description: send an arbitrary message to the jtsn_cmd channel
#  

def take_image():
    p.publish("x")


#   
#   Function: 
#   Description: send an arbitrary message to the jtsn_cmd channel
#   

def say(message):
    p.publish(message)


if __name__ == '__main__':
    try:
        run_publisher()
    except rospy.ROSInterruptException:
        pass