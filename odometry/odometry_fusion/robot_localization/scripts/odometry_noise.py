#!/usr/bin/env python3

import rospy
import numpy as np
import tf
import argparse
import os
import yaml
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry


translation_std = np.array([0.025, 0.025, 0.025]) # m, m, m
angles_std = np.array([0.25, 0.25, 0.25]) # deg, deg, deg


def get_args():
	parser = argparse.ArgumentParser()
	parser.add_argument('--sub_topic', dest='sub_topic', action='store', type=str)
	parser.add_argument('--pub_topic', dest='pub_topic', action='store', type=str)
	args, unknown = parser.parse_known_args()
	return args


def odom_callback(msg):
    global odom_pub, T_l2c, base_odom
    # step 1
    quaternion = (
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w
    )
    translation = (
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    )

    # step 2
    rot = R(quaternion)
    angles = rot.as_euler('zyx', degrees=True)

    # step 3
    noisy_translation = []
    for i in range(3):
        noisy_translation.append(translation[i] + np.random.normal(0, translation_std[i]))

    noisy_angles = []
    for i in range(3):
        noisy_angles.append(angles[i] + np.random.normal(0, angles_std[i]))

    # step 4 
    noisy_rot = R.from_euler('zyx', noisy_angles, degrees=True)
    noisy_quaternion = noisy_rot.as_quat()

    # step 5
    msg.pose.pose.orientation.x = noisy_quaternion[0]
    msg.pose.pose.orientation.y = noisy_quaternion[1]
    msg.pose.pose.orientation.z = noisy_quaternion[2]
    msg.pose.pose.orientation.w = noisy_quaternion[3]

    msg.pose.pose.position.x = noisy_translation[0]
    msg.pose.pose.position.y = noisy_translation[1]
    msg.pose.pose.position.z = noisy_translation[2]   

    odom_pub.publish(msg)


def main(args):
	global odom_pub, odom_sub
	# subscribe and public
	rospy.init_node("add_noise_from_" + args.sub_topic.replace('/', ''))
	odom_sub = rospy.Subscriber(args.sub_topic, Odometry, odom_callback)
	odom_pub = rospy.Publisher(args.pub_topic, Odometry, queue_size=10)
	rospy.spin()


if __name__ == '__main__':
	args = get_args()
	main(args)

