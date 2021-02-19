import rospy
import numpy as np
#import tf
import argparse
import os
import yaml
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry


T_frame1_to_frame2 = None
base_odom = None
args_ = None


def get_args():
    parser = argparse.ArgumentParser()
    parser.add_argument('--nodename', dest='nodename', action='store', type=str)
    parser.add_argument('--sub_topic', dest='sub_topic', action='store', type=str)
    parser.add_argument('--pub_topic', dest='pub_topic', action='store', type=str)
    parser.add_argument('--frame_id', dest='frame_id', action='store', type=str, default='odom')
    parser.add_argument('--child_frame_id', dest='child_frame_id', action='store', type=str)
    parser.add_argument('--calib_filename', dest='calib_filename', action='store', type=str)
    parser.add_argument('--inverse', dest='inverse', action='store_true')
    parser.add_argument('--reset_first', dest='reset_first', action='store_true')
    args, unknown = parser.parse_known_args()
    return args


def odom_callback(msg):
    global odom_pub, T_frame1_to_frame2, base_odom, args_
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

    rot = R(quaternion)
    pose = np.identity(4)
    pose[:3, :3] = rot.as_matrix()
    pose[:3, 3] = translation

    if args.inverse:
        T_frame2_to_frame1 = np.linalg.inv(T_frame1_to_frame2)
        pose = T_frame2_to_frame1.dot(pose).dot(np.linalg.inv(T_frame2_to_frame1))
    else:
        pose = T_frame1_to_frame2.dot(pose).dot(np.linalg.inv(T_frame1_to_frame2))


    if args.reset_first:
        if base_odom is None:
            base_odom = np.copy(pose)
        pose = np.linalg.inv(base_odom).dot(pose)


    rot = R.from_matrix(pose[:3, :3])
    quaternion = rot.as_quat()
    translation = pose[:3, 3]

    msg.pose.pose.orientation.x = quaternion[0]
    msg.pose.pose.orientation.y = quaternion[1]
    msg.pose.pose.orientation.z = quaternion[2]
    msg.pose.pose.orientation.w = quaternion[3]

    msg.pose.pose.position.x = translation[0]
    msg.pose.pose.position.y = translation[1]
    msg.pose.pose.position.z = translation[2]  

    msg.header.frame_id = args.frame_id
    msg.child_frame_id = args.child_frame_id 

    odom_pub.publish(msg)


def main(args):
    global T_frame1_to_frame2, odom_pub, odom_sub, args_
    # get transform
    with open(args.calib_filename) as calib_file:
    	T_frame1_to_frame2 = yaml.load(calib_file, Loader=yaml.FullLoader)['T']
    	T_frame1_to_frame2 = np.array(T_frame1_to_frame2)
    # subscribe and public
    rospy.init_node(args.nodename)
    odom_sub = rospy.Subscriber(args.sub_topic, Odometry, odom_callback)
    odom_pub = rospy.Publisher(args.pub_topic, Odometry, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
	args = get_args()
	main(args)

