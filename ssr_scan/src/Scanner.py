#!/usr/bin/python
#   Scanner.py
#   
#   Author: Logan Yoder
#   
#   Description: Team 1's movement script

import sys
import os
import MODParser as mp
import rospy
import numpy as np
import moveit_commander
from geometry_msgs.msg import Imu
from sensor_msgs.msg import Image
from PIL import Image

def move_point(move_group, pose):
    # Create empty pose obejct which is what MoveIt needs to execute a move
    pose_goal = geometry_msgs.msg.Pose()

    # Separate the pose array into quaternions and positions
    quaternions = np.array(pose[1])
    position = np.array(pose[0])

    # Map position and quaternions to the pose_goal message, 
    pose_goal.orientation.x = quaternions.item(0)
    pose_goal.orientation.y = quaternions.item(1)
    pose_goal.orientation.z = quaternions.item(2)
    pose_goal.orientation.w = quaternions.item(3)

    # NOTE: Positions must be in meters, MOD file has them in mm hence the /1000
    pose_goal.position.x = position.item(0) / 1000
    pose_goal.position.y = position.item(1) / 1000
    pose_goal.position.z = position.item(2) / 1000
    
    # Execute move, wait for completion, then stop movement
    move_group.set_pose_target(pose_goal)
    move_group.go(wait=True)
    move_group.stop()

def scan(filepath, out_path):
    # initialiaze moveit commander to interface with robot
    moveit_commander.roscpp_initialize(sys.argv)

    # create a node for this script
    rospy.init_node("ssr_scan", anonymous=True)

    # Get camera calibration objects
    rgb_caminfo_msg = rospy.wait_for_message("/rgb/camera_info")
    depth_caminfo_msg = rospy.wait_for_message("/depth/camera_info")

    # TODO: Write the calibration information to a file
    
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

    poses = mp.read_file(filepath=filepath) 

    # Counter for file names
    ii = 0

    for pose in poses:
        move_point(move_group, pose)
        # TODO: Revisit this later, checking IMU accel data to make sure camera is no longer moving
        # acceleration = 0
        # while (math.abs(acceleration - (-9.81)) > 1e-3):
        #     imu_msg = rospy.wait_for_message("imu")
        #     acceleration = math.sqrt(imu_msg.linear_acceleration.x**2 + imu_msg.linear_acceleration.y**2 + imu_msg.linear_accleration.z**2)        

        # Get rgb and depth images
        rgb_image_msg = rospy.wait_for_message("/rgb/image_raw")
        depth_image_msg = rospy.wait_for_message("/depth/image_raw")

        # Write images to files
        rgb_img = Image.fromarray(np.array(rgb_image_msg.data)).convert('RGB')
        rgb_img.save("/output/rgb/{}_rgb.jpg".format(ii))

        depth_img = Image.fromarray(np.array(depth_image_msg.data))
        depth_img.save('/output/depth/{}_depth.png'.format(ii))

        move_group.clear_pose_targets()

if __name__ == '__main__':

    # TODO: Add more flexibility to the user input
    # scan you want to run must be at '/input/scan.mod' inside container
    filepath = os.path.abspath('/input/scan.mod')
    output_path = os.path.abspath('/output')

    try:
        scan(filepath, output_path)
    except rospy.ROSInterruptException:
        pass
