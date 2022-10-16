#!/usr/bin/python
import sys
import copy
import os
import MODParser as mp
import numpy as np
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg

def scan(filepath, out_path):
    print ("====initializing setup====")
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("ssr_scan", anonymous=True)

    robot = moveit_commander.RobotCommander()

    scenen = moveit_commander.PlanningSceneInterface()

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
    
    group.set_goal_tolerance(0.5)
    
    group.set_planning_time(10)
    
    group.set_pose_target(pose_target)

    group.go(wait=True)
    
    group.clear_pose_targets()
if __name__ == '__main__':

    filepath = os.path.abspath('/home/sebastian/Downloads/scanComp_3_1000_45_T_ROB1.mod')
    out_path = os.path.abspath('/output/')

    try:
        scan(filepath,out_path)
    except rospy.ROSInterruptException:
        pass


