#!/usr/bin/python

import rospy
import numpy
from message_filters import ApproximateTimeSynchronizer, Subscriber

from geometry_msgs.msg import Pose
from camera_objects_msgs.msg import ObjectArray, Object, RLE

#####
# test pipeline resize  left camera info 1280 720
# rect module?
# bag rect or raw
# only 1 object
#####


buttons = {"left": None, "right": None}
#distance between cameras
T = 0.12
#info from camera info topic
fx = 682.7612915039062
fy = 682.7612915039062

c_x = 615.1388549804688

c_y = 345.3089599609375


def produce_center_point(roi):
    return int(roi.x_offset + roi.width / 2), int(roi.y_offset + roi.height / 2)

def to_manipulator(pose):
    lc = pose.position
    lc_coord = numpy.array([[lc.x], [lc.y], [lc.z]])
    T = [[0], [0], [0]]
    R_lc2mp = numpy.ones([[1, 0, 0],
                             [0, 1, 0],
                             [0, 0, 1]])
    mp_coord = numpy.linalg.inv(R) @ (lc_coord - T)
    pose.position.x = mp_coord[0][0]
    pose.position.y = mp_coord[1][0]
    pose.position.z = mp_coord[2][0]

def callback(data_l, data_r):
    #simetric objects
    if data_l.object and data_r.object:
        bbox = data_l.objects[0].bbox
        time = data_l.header.stamp.secs
        buttons["left"] = {"seq": data_l.header.seq, "time": time, "xy": produce_center_point(bbox)}

        bbox = data_r.objects[0].bbox
        time = data_r.header.stamp.secs
        buttons["right"] = {"seq": data_l.header.seq, "time": time, "xy": produce_center_point(bbox)}
    elif not data_l.object:
        rospy.loginfo("not l")
    elif not data_r.object:
        rospy.loginfo("not r")

if __name__ == '__main__':
    rospy.init_node('xyz')

    tss = ApproximateTimeSynchronizer([Subscriber('/zed_node/left/objects', ObjectArray), Subscriber('/zed_node/right/objects', ObjectArray)], queue_size=10, slop=0.05)
    tss.registerCallback(callback)
    
    pub = rospy.Publisher('/zed_node/buttonPose', Pose, queue_size=10)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():

        
        if not buttons["left"] or not buttons["right"]:
            continue
        else:
            rospy.loginfo(buttons)
                # position float, quant - normalized
                #by left camera
                # with rect
            try:
                Z = round(float(fx * T / (abs(buttons["left"]["xy"][0] - buttons["right"]["xy"][0]))), 3)
            except ZeroDivisionError:
                continue
            pose = Pose()
            pose.position.x = round(float((buttons["left"]["xy"][0] - c_x) * Z / fx), 3)
            pose.position.y = round(float((buttons["left"]["xy"][1] - c_y) * Z / fy), 3)
            pose.position.z = Z
            pub.publish(pose)

            buttons["left"] = None
            buttons["right"] = None

        rate.sleep()
