#!/usr/bin/python

#   MoveRob.py
#   
#   Author of original: Logan Yoder
#
#   Contributors: Tom Geveke, Sebastian Schweitzer, Dean Taipale
#   
#   Description: Team 2's movement script


import sys
import copy
import os
import MODParser as mp
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg


import JtsnCommander as JC


#   
#   Function: scan
#   Parameters: filepath - 
#               out_path - 
#   Description: Scan Initializes the Robot,Zeros the joints, and then reads the input file.
#   

def scan(filepath, out_path):
    print ("====initializing setup====")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ssr_scan", anonymous=True)

    robot = moveit_commander.RobotCommander()

    scene = moveit_commander.PlanningSceneInterface()

    group = moveit_commander.MoveGroupCommander("manipulator")


    
    joint_goal =group.get_current_joint_values()
    
    joint_goal[0]=0
    joint_goal[1]=0
    joint_goal[2]=0
    joint_goal[3]=0
    joint_goal[4]=0
    joint_goal[5]=0
    
    group.go(wait=True)
    group.stop()
    poses = mp.read_file(filepath)

    for pose in poses:
        move(group, pose)

        JC.take_image()
        

#   
#   Function: move
#   Description: Move function uses MODparser to fill the quaternion and xyz values arrays.
#   It then sets the pose and increments the array position.
#   

def move(group, pose):   	
    pose_target = geometry_msgs.msg.Pose()
	

    quaternions= np.array(pose[1])
    position = np.array(pose[0])
    print ("Quaternion: ", quaternions)
    print ("Positions: ",position)
    	
    pose_target.position.x = position.item(0)/1000 
    pose_target.position.y = position.item(1)/1000 
    pose_target.position.z = position.item(2)/1000 
    
    
    pose_target.orientation.x = quaternions.item(0)
    pose_target.orientation.y = quaternions.item(1)
    pose_target.orientation.z = quaternions.item(2)
    pose_target.orientation.w = quaternions.item(3)
    
    

    
    group.set_pose_target(pose_target)

    group.go(wait=True)
    
    group.clear_pose_targets()


# main method
if __name__ == '__main__':

    #todo: switch to globals?
    filepath = os.path.abspath('')
    out_path = os.path.abspath('/output/')

    try:
        scan(filepath,out_path)


    except rospy.ROSInterruptException:
        pass


