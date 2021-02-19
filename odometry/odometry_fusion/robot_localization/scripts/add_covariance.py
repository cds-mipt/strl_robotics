#!/usr/bin/env python

import argparse
import numpy as np
import rospy
from nav_msgs.msg import Odometry


def callback(data):
	global args, odom_pub, covariance
	data.pose.covariance = covariance
	if args.frame_id is not None:
		data.header.frame_id = args.frame_id
	if args.child_frame_id is not None:
		data.child_frame_id = args.child_frame_id 
	odom_pub.publish(data)


def main():
	global args, odom_pub, odom_sub, covariance

	parser = argparse.ArgumentParser(description='Add covariance to Odometry')
	parser.add_argument('--cov_filename', dest='cov_filename', action='store', type=str)
	parser.add_argument('--nodename', dest='nodename', action='store', type=str)
	parser.add_argument('--sub_topic', dest='sub_topic', action='store', type=str)
	parser.add_argument('--pub_topic', dest='pub_topic', action='store', type=str)
	parser.add_argument('--frame_id', dest='frame_id', action='store', type=str, default=None)
	parser.add_argument('--child_frame_id', dest='child_frame_id', action='store', type=str, default=None)
	args = parser.parse_args()

	covariance = np.genfromtxt(args.cov_filename)

	rospy.init_node(args.nodename)
	odom_sub = rospy.Subscriber(args.sub_topic, Odometry, callback)
	odom_pub = rospy.Publisher(args.pub_topic, Odometry, queue_size=10)
	rospy.spin()


if __name__ == '__main__':
	main()
