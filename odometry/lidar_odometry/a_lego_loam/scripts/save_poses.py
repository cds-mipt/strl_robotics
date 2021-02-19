#!/usr/bin/env python

# This script saves poses in TUM format
# TUM format: timestamp, tx, ty, tz, qx, qy, qz, qw

from __future__ import print_function, division, with_statement

import errno
import rospy
import numpy as np
import tf
import argparse
import os
import yaml

from nav_msgs.msg import Odometry

OUTFILE = None


def mkdir(p):
    try:
        os.makedirs(p)
    except OSError as e:
        if e.errno != errno.EEXIST:
            raise


def build_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument('-t', '--topic', type=str, required=True)
    parser.add_argument('-o', '--output', type=str, required=True)
    return parser


def callback(msg):
    global OUTFILE
    timestamp = msg.header.stamp.to_sec()
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

    m = [timestamp] + list(translation) + list(quaternion)
    with open(OUTFILE, 'a') as f:
        f.write(' '.join(map(lambda x: '{:.18e}'.format(x), m)) + '\n')


def main(args):
    global OUTFILE

    outfolder = os.path.join(os.path.dirname(args.output), args.topic[1:].replace('/', '_'))
    mkdir(outfolder)
    OUTFILE = os.path.join(outfolder, os.path.basename(args.output))
    print('Saving poses to', os.path.abspath(OUTFILE))

    rospy.init_node('record_lego_loam_odometry')
    rospy.Subscriber(args.topic, Odometry, callback)
    rospy.spin()


if __name__ == '__main__':
    parser = build_parser()
    args, unknown = parser.parse_known_args()
    main(args)
