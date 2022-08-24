#!/usr/bin/env python3
#in this version of the function the coordinates of the button are read from the button location topic
import numpy as np
import time
import roslib; roslib.load_manifest('ur_driver')
import rospy, sys
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
from control_msgs.msg import *
from trajectory_msgs.msg import *
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from math import pi
from moveit_commander import MoveGroupCommander
from copy import deepcopy	

from manipulator.srv import GetButtons



if __name__ == '__main__':

    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node("press_button", anonymous=True, disable_signals=True)
    cartesian = rospy.get_param('~cartesian', True)
    print "cartesian = %s" % cartesian

    # Connect to the manipulator move group
    manipulator = MoveGroupCommander('manipulator')

    # Allow replanning to increase the odds of a solution
    manipulator.allow_replanning(True)

    #Set planner from OMPL library
    manipulator.set_planner_id("TRRTkConfigDefault")

    # Set the right arm reference frame
    manipulator.set_pose_reference_frame('base_link')

    # Allow some leeway in position(meters) and orientation (radians)
    manipulator.set_goal_position_tolerance(0.001)
    manipulator.set_goal_orientation_tolerance(0.001)

    manipulator.set_max_acceleration_scaling_factor(0.0625)

    # Get current Joint Position
    joint_pose = manipulator.get_current_joint_values()
    #print joint_pose

    # Get current Cartesian Pose
    end_effector_link = manipulator.get_end_effector_link()
    cart_pose = manipulator.get_current_pose(end_effector_link).pose
    #print cart_pose
    #print "%s" % end_effector_link

    # Get name of current planner
    current_planner = manipulator.get_planner_id()
    #print "%s" % current_planner

    # Get the button center coordinate
    print "wait for service"
    rospy.wait_for_service('get_buttons_pose')
    try:
        get_buttons_pose = rospy.ServiceProxy('get_buttons_pose', GetButtons)
        pose = get_buttons_pose(1)

    except rospy.ServiceException, e:
        print "Service call failed: %s"%e

    # Place for coordinate transformation


    #button_location_as_list = [0.88888888, 0.4, 0.985, 0.0001, 0.0001, 0.0001, 0.9999] # Different x&y 30-31/03/2021
    #button_location = moveit_commander.conversions.list_to_pose(button_location_as_list)

    button_location = deepcopy(pose.pose)
    # coordinate transformation for perpendicular robot
    #button_location.position.x += -0.17
    #button_location.position.y += 0.11
    #button_location.position.z += 0.02
    #button_location.orientation.x = 0
    #button_location.orientation.y = 0
    #button_location.orientation.z = 0
    #button_location.orientation.w = 1


    # Specify the point in front of the button
    #in_front_as_list = [0.87, 0.1135, 0.9786, 0.0001, 0.0001, 0.0001, 0.9999]
    #in_front = moveit_commander.conversions.list_to_pose(in_front_as_list)
    in_front = deepcopy(button_location)
    in_front.position.x -= 0.05

    # move to the point in front of the button
    joint_pose_1 = manipulator.get_current_joint_values()
    print joint_pose_1
    #manipulator.set_start_state_to_current_state()
    manipulator.set_pose_target(in_front)
    plan_in_front = manipulator.plan()
    print "current target:"
    print in_front
    inp = raw_input("Move to the point in front of the button? y/n: ")[0]
    if (inp == 'y'):
        manipulator.execute(plan_in_front)
    else:
       print "Skipping"
    
    #Reduce maximum velocity for button pushing
    #manipulator.set_max_velocity_scaling_factor(0.0625)
    manipulator.set_max_acceleration_scaling_factor(0.0625)


    #Specify point when button is pushed
    #pushed_as_list = [0.93, 0.1135, 0.9786, 0.0001, 0.0001, 0.0001, 0.9999]
    #pushed = moveit_commander.conversions.list_to_pose(pushed_as_list)
    pushed = deepcopy(button_location)
    pushed.position.x += 0.003

    #push the button
    joint_pose_2 = manipulator.get_current_joint_values()
    print joint_pose_2
    #manipulator.set_start_state_to_current_state()
    
    # Simple planning
    manipulator.set_pose_target(pushed)
    plan_push = manipulator.plan()
    
    # Planning motion along the straight line. In practice is less smooth
    #waypoints = []
    #waypoints.append(manipulator.get_current_pose().pose)
    #waypoints.append(pushed)
    #(plan_push, fraction) = manipulator.compute_cartesian_path(waypoints, 0.01, 0.0)

    print "current target:"
    print pushed
    inp = raw_input("Push the button? y/n: ")[0]
    if (inp == 'y'):
        manipulator.execute(plan_push)
    else:
        print "Skipping"

    #Specify point when button is released
    #released_as_list = [0.9, 0.1135, 0.9786, 0.0001, 0.0001, 0.0001, 0.9999]
    #released = moveit_commander.conversions.list_to_pose(released_as_list)
    released = deepcopy(button_location)
    released.position.x -= 0.04

    #release the button
    joint_pose_3 = manipulator.get_current_joint_values()
    print joint_pose_3
    #manipulator.set_start_state_to_current_state()
    manipulator.set_pose_target(released)
    plan_release = manipulator.plan()
    print "current target:"
    print released
    inp = raw_input("Release the button? y/n: ")[0]
    if (inp == 'y'):
        manipulator.execute(plan_release)
    else:
        print "Skipping"

    #Specify position when robot is folded
    #folded_as_list = [0.452, 0.109, 0.587, 0.0001, 0.0001, 0.0001, 0.9999]
    #folded = moveit_commander.conversions.list_to_pose(folded_as_list)
    folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
    
    #fold the robot
    joint_pose_4 = manipulator.get_current_joint_values()
    print joint_pose_4
    #manipulator.set_start_state_to_current_state()
    #manipulator.set_pose_target(folded)
    manipulator.set_joint_value_target(folded_joints)
    plan_fold = manipulator.plan()
    inp = raw_input("Fold the robot y/n: ")[0]
    if (inp == 'y'):
        manipulator.execute(plan_fold)
    else:
        print "Skipping"




