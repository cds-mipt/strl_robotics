#!/usr/bin/env python3
import rospy
import tf2_ros
import tf2_geometry_msgs
from tf.transformations import quaternion_from_euler
import geometry_msgs.msg
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
import cv2
from cv_bridge import CvBridge
import numpy as np

    
def draw_occupancy_grid(occ_grid, frame=None):
    width = occ_grid.info.width
    height = occ_grid.info.height
    image = np.zeros((height, width, 3), np.uint8)
    data_iter = 0
    for j in range(height-1, -1, -1):
        for i in range(width):
            occ_grid_value = occ_grid.data[data_iter]
            if occ_grid_value == -1:
                pixel_value = 100
            else:
                pixel_value = int((100 - occ_grid_value) * 255 / 100)
            image[j, i] = (pixel_value, pixel_value, pixel_value)
            data_iter += 1
    
    k = 1
    if frame:
        k = min(frame[0] / width, frame[1] / height)
        image = cv2.resize(image, (int(width * k), int(height * k)))
    return image, k
    
    
def get_robot_world_pose(desired_time=None):
    global tfBuffer
    if desired_time is None:
        desired_time = rospy.Time(0)
    try:
        map_to_base_link = tfBuffer.lookup_transform('map', 'base_link', desired_time, rospy.Duration(1.0))
    except:
        print('Could not find transform from map to base_link')
        return None
    robot_world_pose = geometry_msgs.msg.PoseStamped()
    robot_world_pose.pose.position = map_to_base_link.transform.translation
    robot_world_pose.pose.orientation = map_to_base_link.transform.rotation
    return robot_world_pose
    
    
def inverse_transform(transform):
    inversed_transform = geometry_msgs.msg.TransformStamped()
    
    inversed_rotation = geometry_msgs.msg.TransformStamped()
    inversed_rotation.transform.translation = geometry_msgs.msg.Vector3(0, 0, 0)
    inversed_rotation.transform.rotation = transform.transform.rotation
    inversed_rotation.transform.rotation.w *= -1
    
    inversed_translation = geometry_msgs.msg.Vector3Stamped()
    inversed_translation.vector = transform.transform.translation
    inversed_translation.vector.x *= -1
    inversed_translation.vector.y *= -1
    inversed_translation.vector.z *= -1
    inversed_translation = tf2_geometry_msgs.do_transform_vector3(inversed_translation, inversed_rotation)
    
    inversed_transform.transform.translation = inversed_translation.vector
    inversed_transform.transform.rotation = inversed_rotation.transform.rotation
    return inversed_transform


def transform_world_pose_to_occupancy_grid_pose(world_pose, occ_grid_info):
    transform = geometry_msgs.msg.TransformStamped()
    transform.transform.translation = occ_grid_info.origin.position
    transform.transform.rotation = occ_grid_info.origin.orientation
    inversed_transform = inverse_transform(transform)
    
    occ_grid_pose = tf2_geometry_msgs.do_transform_pose(world_pose, inversed_transform)
    return occ_grid_pose


def draw_pose(image, k, robot_pose, occ_grid_info):
    x = robot_pose.pose.position.x * k / occ_grid_info.resolution
    y = robot_pose.pose.position.y * k / occ_grid_info.resolution
    i = int(x)
    j = int(image.shape[0] - y)
    angle = 2 * np.arccos(robot_pose.pose.orientation.w) * np.sign(robot_pose.pose.orientation.z)  # only for 3 DoF movements
    
    radius = 6
    length = 15
    width = 3
    cv2.circle(image, (i, j), radius, (0, 0, 255), -1)
    dx = length * np.cos(angle)
    dy = length * np.sin(angle)
    di = int(dx)
    dj = int(-dy)
    cv2.line(image, (i, j), (i+di, j+dj), (0, 0, 255), width)
    
    
def process_occupancy_grid(occ_grid):
    global bridge
    global pub
    image, k = draw_occupancy_grid(occ_grid, frame=(1500, 1000))
    robot_world_pose = get_robot_world_pose(desired_time=occ_grid.header.stamp)
    if robot_world_pose is None:
        return()
    robor_occ_grid_pose = transform_world_pose_to_occupancy_grid_pose(robot_world_pose, occ_grid.info)
    draw_pose(image, k, robor_occ_grid_pose, occ_grid.info)
    image_message = bridge.cv2_to_imgmsg(image, encoding="passthrough")
    image_message.header.stamp = occ_grid.header.stamp
    pub.publish(image_message)


if __name__ == '__main__':
    rospy.init_node('occupancy_grid_demo')
    tfBuffer = tf2_ros.Buffer()
    tfListener = tf2_ros.TransformListener(tfBuffer)
    bridge = CvBridge()
    rospy.Subscriber('/grid_map', OccupancyGrid, process_occupancy_grid)
    pub = rospy.Publisher('/occupancy_grid_demo', Image, queue_size=10)
    rospy.spin()

