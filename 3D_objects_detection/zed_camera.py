#!/usr/bin/python

import rospy
import numpy
from message_filters import ApproximateTimeSynchronizer, Subscriber

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from camera_objects_msgs.msg import ObjectArray, Object, RLE


#info from camera info topic
fx = 682.7612915039062
fy = 682.7612915039062

c_x = 615.1388549804688
c_y = 345.3089599609375


def produce_center_point(roi):
    #simetric object
    return int(roi.x_offset + roi.width / 2), int(roi.y_offset + roi.height / 2)

def to_manipulator(pose):
    lc = pose.position
    lc_coord = numpy.array([[lc.x], [lc.y], [lc.z]])
    T = [[0], [0], [0]]
    R_lc2mp = numpy.ones([[1, 0, 0],
                          [0, 1, 0],
                          [0, 0, 1]])
    mp_coord = np.matmul(numpy.linalg.inv(R), (lc_coord - T))
    pose.position.x = mp_coord[0][0]
    pose.position.y = mp_coord[1][0]
    pose.position.z = mp_coord[2][0]

def callback(data_l, depth_l):
    if data_l.objects:
        bbox = data_l.objects[0].bbox
        x, y = produce_center_point(bbox)

        rospy.loginfo("Looking for", x, y)
        rospy.loginfo("Get", depth_l.width, depth_l.height, "depth image")
        
        depths = numpy.array(depth_l.data[]).reshape((depth_l.height, depth_l.width))
        depths = depths[y-bbox.height/2:y+bbox.height/2][x-bbox.width/2:x+bbox.width]

        #чтобы оценить какие значение отвечают за нераспознанные 
        #unique, counts = numpy.unique(a, return_counts=True)
        #rospy.loginfo(dict(zip(unique, counts))) 

        try:
            Z = round(float(np.median(depths)), 3)
        except:
            pass

        pose = Pose()
        pose.position.x = round(float((x - c_x) * Z / fx), 3)
        pose.position.y = round(float((y - c_y) * Z / fy), 3)
        pose.position.z = Z
        pub.publish(pose)

    else:
        rospy.loginfo("not l objects")


if __name__ == '__main__':
    rospy.init_node('xyz')

    tss = ApproximateTimeSynchronizer([Subscriber('/zed_node/left/objects', ObjectArray), 
				       Subscriber('/zed/zed_node/depth/depth_registered', Image)], 
                                       queue_size=10, 
                                       slop=1)
    tss.registerCallback(callback)
    
    pub = rospy.Publisher('/zed_node/buttonPose_from_Dsensor', Pose, queue_size=10)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        rate.sleep()
