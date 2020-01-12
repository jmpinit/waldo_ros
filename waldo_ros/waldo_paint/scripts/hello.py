#!/usr/bin/env python

import sys
import copy
import math
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import tf_conversions
from std_msgs.msg import String

def scale_pt(pt):
    x, y = pt
    return (x / 1000.0, y / 1000.0)

# Get pose oriented toward canvas with initial_pose as origin
def painting_pose(initial_pose, x, y, z):
    wpose = geometry_msgs.msg.Pose()

    #ten_deg = 10.0 / 360.0 * 2.0 * math.pi
    #roll = 0
    #pitch = math.pi / 2
    #yaw = -math.pi / 2
    #wpose.orientation = geometry_msgs.msg.Quaternion(*tf_conversions.transformations.quaternion_from_euler(roll, pitch, yaw))

    wpose.orientation.x = initial_pose.orientation.x
    wpose.orientation.y = initial_pose.orientation.y
    wpose.orientation.z = initial_pose.orientation.z
    wpose.orientation.w = initial_pose.orientation.w

    wpose.position.x = initial_pose.position.x + x
    wpose.position.y = initial_pose.position.y + y
    wpose.position.z = initial_pose.position.z + z

    return copy.deepcopy(wpose)

def follow_waypoints(group, waypoints):
    # Do it!
    eef_step = 0.05 # Resolution of path in meters
    jump_threshold = 0 # Disable jumps
    painting_plan, fraction = group.compute_cartesian_path(waypoints, eef_step, jump_threshold)
    group.execute(painting_plan, wait=True)

def dip_brush(initial_pose, group):
    waypoints = []

    start_pose = group.get_current_pose().pose
    
    # Move away from canvas
    waypoints.append(painting_pose(start_pose, 0, 0, 0.2))

    # Move over paint 
    waypoints.append(painting_pose(initial_pose, -0.1, -0.1, 0.2))

    # Move into paint 
    waypoints.append(painting_pose(initial_pose, -0.1, -0.1, 0))

    # Move over paint 
    waypoints.append(painting_pose(initial_pose, -0.1, -0.1, 0.2))

    # Move back to where we were
    waypoints.append(painting_pose(start_pose, 0, 0, 0.2))

    follow_waypoints(group, waypoints)

def paint_path(initial_pose, group, path):
    offset = 0.01
    waypoints = []

    # Start at current pose
    current_pose = group.get_current_pose().pose
    waypoints.append(current_pose)

    def move_to(x, y, z):
        waypoints.append(painting_pose(initial_pose, x, y, z))

    # Move to above start
    start_x, start_y = scale_pt(path[0])
    move_to(start_x, start_y, offset)

    # To canvas
    move_to(start_x, start_y, 0)

    for x, y in map(scale_pt, path):
        move_to(x, y, 0)

    # Away from canvas
    last_x, last_y = scale_pt(path[-1])
    move_to(last_x, last_y, offset)

    follow_waypoints(group, waypoints)

def paint_paths(paths):
    moveit_commander.roscpp_initialize(sys.argv)

    robot = moveit_commander.RobotCommander()
    group = moveit_commander.MoveGroupCommander('arm')

    initial_pose = group.get_current_pose().pose

    for path in paths:
        dip_brush(initial_pose, group)
        rospy.sleep(0.5)
        paint_path(initial_pose, group, path)
        rospy.sleep(0.5)

    dip_brush(initial_pose, group)
    rospy.sleep(0.5)

    group.set_pose_target(initial_pose)
    group.go(wait=True)

    moveit_commander.roscpp_shutdown()

def paint_circle(centerX, centerY, centerZ):
    circle = []

    count = 32
    r = 0.1
    for i in range(0, count):
        angle = float(i) / count * math.pi * 2
        x = centerX + r * math.cos(angle)
        y = centerY + r * math.sin(angle)
        circle.append([x, y])

    paint_paths([circle])

if __name__ == '__main__':
    rospy.init_node('hello')

    try:
        print('Attempting to paint a circle...')
        paint_circle(0, 0, 0)
        print('Done')
    except rospy.ROSInterruptException:
        print('Failed to paint circle')
        pass

