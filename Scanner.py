#!/usr/bin/python3
import sys
import argparse
import os
from time import sleep
from tokenize import group
import MODParser as mp
import rospy
from std_msgs.msg import String
import numpy as np
import moveit_commander
import geometry_msgs.msg
import math


def scan(fp):
    # initialiaze moveit commander to interface with robot
    moveit_commander.roscpp_initialize(sys.argv)

    # create a node for this script
    rospy.init_node("ssr_scan", anonymous=True)
    
    # Create a robot object
    robot = moveit_commander.RobotCommander()

    # Create a scene object: this allows you to add objects for collision checking
    scene = moveit_commander.PlanningSceneInterface()
    
    # This is the name of the joint group for the ABB IRB 2400 from the 
    # ABB ROS package moveit config
    group_name = "manipulator"

    # Isolating the move group object which allows us to get the functions we
    # care about like "plan" and "go" for movement
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # setting up a subscriber to the AK RGB image topic 
    rgb_subscriber = rospy.Subscriber(
        "/rgb/image_raw"
    )

    depth_subscriber = rospy.Subscriber(
        "/depth/image_raw",

    )

    joint_goal = move_group.get_current_joint_values()

    # Move to "Home/All-Zeros" position
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0

    move_group.go(wait=True) # Execute the move command
    move_group.stop()

    poses = mp.read_file(filepath=fp) 


    for pose in poses:
        pose_goal = geometry_msgs.msg.Pose()
        quaternions = np.array(pose[1])
        position = np.array(pose[0])
        pose_goal.orientation.x = quaternions.item(0) / 1000
        pose_goal.orientation.y = quaternions.item(1) / 1000
        pose_goal.orientation.z = quaternions.item(2) / 1000
        pose_goal.orientation.w = quaternions.item(3) / 1000
        pose_goal.position.x = position.item(0)
        pose_goal.position.y = position.item(1)
        pose_goal.position.z = position.item(2)
        
        move_group.set_pose_target(pose_goal)
        move_group.go(wait=True)
        move_group.stop()

        #TODO:Grab AK capture from the AK topic
            
        move_group.clear_pose_targets()

if __name__ == '__main__':
    # Initialize commandline argument parser
    parser = argparse.ArgumentParser(
        description="SSR scanning program"
    ) 

    # Add parameter(s)
    parser.add_argument('input_file', help="Pass filepath to MOD file for scanning")

    # Parse the arguments
    args = parser.parse_args()

    filepath = os.path.abspath(args.input_file[0])

    try:
        scan(filepath)
    except rospy.ROSInterruptException:
        pass
