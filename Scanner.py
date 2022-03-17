#!/usr/bin/python3

import sys
from time import sleep
from tokenize import group
import MODParser as mp
import rospy
from std_msgs.msg import String
import numpy as np
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import math


def Scanner():
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("move_group_tutorial", anonymous=True)

    robot = moveit_commander.RobotCommander()
    scene = moveit_commander.PlanningSceneInterface()

    group_name = "manipulator"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    planning_frame = move_group.get_planning_frame()
    print(f"Planning Frame: {planning_frame}")

    display_trajectory_publisher = rospy.Publisher(
    "/move_group/display_planned_path",
    moveit_msgs.msg.DisplayTrajectory,
    queue_size=20,
    )

    eef_link = move_group.get_end_effector_link()
    print(f"End effector link: {eef_link}")
    group_names = robot.get_group_names()

    print(f"Group names: {group_names}")

    tau = 2*math.pi

    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = -tau / 8
    joint_goal[2] = 0
    joint_goal[3] = -tau / 4
    joint_goal[4] = 0
    joint_goal[5] = tau / 6  # 1/6 of a turn

    move_group.go(wait=True)
    move_group.stop()

    poses = mp.read_file(r"/home/logan/ROS/abb_ros/AugmentedRoboticFabrication/T_ROB/doTest_100_T_ROB1.mod") 


    for pose in poses:
        pose_goal = geometry_msgs.msg.Pose()
        quaternions = np.array(pose[1])
        position = np.array(pose[0])
        pose_goal.orientation.x = quaternions.item(0)
        pose_goal.orientation.y = quaternions.item(1)
        pose_goal.orientation.z = quaternions.item(2)
        pose_goal.orientation.w = quaternions.item(3)
        pose_goal.position.x = position.item(0)
        pose_goal.position.y = position.item(1)
        pose_goal.position.z = position.item(2)
        
        print(pose_goal)

        move_group.set_pose_target(pose_goal)
        plan = move_group.plan()

        sleep(20)
        move_group.clear_pose_targets()

if __name__ == '__main__':
    try:
        Scanner()
    except rospy.ROSInterruptException:
        pass
