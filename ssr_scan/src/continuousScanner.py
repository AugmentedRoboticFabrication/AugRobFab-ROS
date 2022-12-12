#!/usr/bin/python3
import sys
import os
import numpy as np
import math
import rospy
import moveit_commander
import geometry_msgs.msg

def parse(fileName):
    try:
        file = open(fileName)
        print('Successfully opened file:', fileName)
    except:
        print('Error opening file:', fileName, 'please enter the correct filename')
        exit()
    lines = file.readlines()
    poses = []
    for line in lines:
        if 'MoveL' in line:
            pose = line.translate(str.maketrans('', '', '[]abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ')).strip()
            splitLine = [x for x in pose.split(',') if x]
            position = splitLine[0:3]
            quat = splitLine[3:]
            quat_right = [quat[1], quat[2], quat[3], quat[0]]
            poses.append((position, quat_right))
    return poses

def getrotx(angle):
    rad = math.radians(angle)
    return [[1, 0,              0],
            [0, math.cos(rad), -math.sin(rad)],
            [0, math.sin(rad),  math.cos(rad)]]

def rpy2quats(pitch, roll, yaw):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    
    return [qw, qx, qy, qz]

def sphere():
    import math

    r = 1
    theta_total = math.radians(90) # Degrees -> radians

    # Center of circle / center of object to scan, in meters
    h = 0
    k = 0.5
    j = 0

    n = 40 # Points per curve
    m = 8 # Number of curves

    xlimit = h + (r * math.cos(theta_total / 2))
    poses = []

    for i in range(-m//2, m//2 + 1):
        x = np.linspace(-xlimit, xlimit, n)
        y = i * (r / m)

        rotx = getrotx(15 * i)
        for ii in range(0, n):
            z = math.sqrt(r**2 - (x[ii] - h)**2)
            oldx = [x[ii], y, z]
            newx = np.matmul(rotx, oldx)

            pos_x = newx[0] + h
            pos_y = newx[1] + k
            pos_z = newx[2] + j

            # plot.scatter(pos_x, pos_y, pos_z)
            # plot.plot3D(np.linspace(pos_x, h, n), np.linspace(pos_y, k, n), np.linspace(pos_z, j, n))
            
            magx = math.sqrt(newx[0]**2 + newx[1]**2 + newx[2]**2)
            xangle = math.acos(np.dot([1, 0, 0], newx) / magx)
            yangle = math.acos(np.dot([0, 1, 0], newx) / magx)
            zangle = math.acos(np.dot([0, 0, 1], newx) / magx)

            quats = rpy2quats(xangle, yangle, zangle)
            poses.append([[pos_x, pos_y, pos_z], quats])
    return poses

def scan(filename):
    # initialiaze moveit commander to interface with robot
    moveit_commander.roscpp_initialize(sys.argv)

    # create a node for this script
    rospy.init_node("ssr_scan", anonymous=True)
    
    # This is the name of the joint group for the ABB IRB 2400 from the ABB ROS package moveit config
    group_name = "manipulator"

    # Isolating the move group object which allows us to get the functions we care about like "plan" and "go" for movement
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Allow replanning to increase the odds of a solution
    move_group.allow_replanning(True)

    # Allow some leeway in position(meters) and orientation (radians)
    move_group.set_goal_position_tolerance(0.01)
    move_group.set_goal_orientation_tolerance(0.1)

    joint_goal = move_group.get_current_joint_values()
    
    # Move to initial position
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = -math.pi
    joint_goal[5] = (7/12) * math.pi

    move_group.go(wait=True) # Execute the move command

    poses = parse(filename)
    # poses = sphere()

    waypoints = []
    for pose in poses:
        # Create empty pose obejct which is what MoveIt needs to execute a move
        print(pose)
        pose_goal = geometry_msgs.msg.Pose()

        # Separate the pose array into quaternions and positions
        quaternions = np.array(pose[1])
        print(quaternions)
        position = np.array(pose[0])
        print(position)

        # Map position and quaternions to the pose_goal message, 
        pose_goal.orientation.x = float(quaternions.item(0))
        pose_goal.orientation.y = float(quaternions.item(1))
        pose_goal.orientation.z = float(quaternions.item(2))
        pose_goal.orientation.w = float(quaternions.item(3))

        # NOTE: Positions must be in meters
        pose_goal.position.x = float(position.item(0)) / 1000
        pose_goal.position.y = float(position.item(1)) / 1000
        pose_goal.position.z = float(position.item(2)) / 1000

        waypoints.append(pose_goal)

    overallpercent = 0 # Variable to store percentage completed to print to console
    for i in range(0, 8): # Loop through each pass of the scan
        print('\n\n_________________________________________________________________________')
        current_points = waypoints[:8] # Gets next 8 points from waypoints list
        print(current_points)
        print(len(current_points)) # Should always be 8

        '''
        # Attempted linear move between end point of last pass and start of next pass, this did not work (gives exit error -11 in RViz terminal)

        move_group.set_pose_target(cur[0])
        move_group.go(wait=True)
        move_group.stop()
        '''

        (plan, percent) = move_group.compute_cartesian_path(current_points, eef_step=0.005, jump_threshold=5.0)
        # print(plan)
        print(100*percent)
        overallpercent += percent

        move_group.execute(plan, wait=True) # Runs the plan returned from compute_cartesian_path above

        waypoints = waypoints[8:] # Removes current_points from waypoints list
        move_group.clear_pose_targets()

    print('Overall Completion Perecent:', round(100 * overallpercent / 8, 3), '%')
    move_group.stop()

if __name__ == '__main__':
    filepath = os.path.abspath('input/scan.mod')
    try:
        scan(filepath)
    except rospy.ROSInterruptException:
        pass
