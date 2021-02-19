#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
import tf2_ros
import tf2_geometry_msgs
import geometry_msgs.msg


def odom_received(odom):
    global tfBroadcaster
    global tfBuffer
    global odom_publisher

    base_link_to_zed_left_camera_optical_frame = tfBuffer.lookup_transform('base_link', 'zed_left_camera_optical_frame', rospy.Time())

    translation = geometry_msgs.msg.Vector3Stamped()
    translation.header.frame_id = 'zed_left_camera_optical_frame'
    translation.vector = odom.pose.pose.position
    translation_base_link = tf2_geometry_msgs.do_transform_vector3(translation, base_link_to_zed_left_camera_optical_frame)

    rotation_vector = geometry_msgs.msg.Vector3Stamped()
    rotation_vector.header.frame_id = 'zed_left_camera_optical_frame'
    rotation_vector.vector.x = odom.pose.pose.orientation.x
    rotation_vector.vector.y = odom.pose.pose.orientation.y
    rotation_vector.vector.z = odom.pose.pose.orientation.z
    rotation_vector_base_link = tf2_geometry_msgs.do_transform_vector3(rotation_vector, base_link_to_zed_left_camera_optical_frame)

    odom_pose_base_link = geometry_msgs.msg.PoseStamped()
    odom_pose_base_link.pose.position = translation_base_link.vector
    odom_pose_base_link.pose.orientation.x = rotation_vector_base_link.vector.x
    odom_pose_base_link.pose.orientation.y = rotation_vector_base_link.vector.y
    odom_pose_base_link.pose.orientation.z = rotation_vector_base_link.vector.z
    odom_pose_base_link.pose.orientation.w = odom.pose.pose.orientation.w

    odom_to_base_link = geometry_msgs.msg.TransformStamped()
    odom_to_base_link.header.stamp = odom.header.stamp
    odom_to_base_link.header.frame_id = 'odom'
    odom_to_base_link.child_frame_id = 'base_link'
    odom_to_base_link.transform.translation = odom_pose_base_link.pose.position
    odom_to_base_link.transform.rotation = odom_pose_base_link.pose.orientation
    tfBroadcaster.sendTransform(odom_to_base_link)

    odom_base_link = odom
    odom_base_link.header.frame_id = 'odom'
    odom_base_link.pose.pose = odom_pose_base_link.pose
    odom_publisher.publish(odom_base_link)


if __name__ == '__main__':
    rospy.init_node('openvslam_odom_to_base_link')
    tfBroadcaster = tf2_ros.TransformBroadcaster()
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    rospy.Subscriber('/OpenVSLAM/odom', Odometry, odom_received)
    odom_publisher = rospy.Publisher('/OpenVSLAM/odom_base_link', Odometry, queue_size=10)
    rospy.spin()
    
