#!/usr/bin/python

import rospy
import numpy
from message_filters import ApproximateTimeSynchronizer, Subscriber

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Image
from camera_objects_msgs.msg import ObjectArray, Object, RLE

from cv_bridge import CvBridge
bridge = CvBridge()

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
    T = [-0.2388, -0.06, 0.75]
    R_cl2b = numpy.array([[0, 0, 1],
                           [-1, 0, 0],
                           [0, -1, 0]])
    mp_coord = numpy.matmul(R_cl2b, lc_coord)
    pose.position.x = mp_coord[0][0] + T[0]
    pose.position.y = mp_coord[1][0] + T[1]
    pose.position.z = mp_coord[2][0] + T[2]
    return pose

def callback(data_l, depth_l):
    if data_l.objects:
        bbox = data_l.objects[0].bbox
        x, y = produce_center_point(bbox)

        #print("Looking for", x, y)
        #print("Get", depth_l.width, depth_l.height, "depth image")
        

        try:
            depths = bridge.imgmsg_to_cv2(depth_l, desired_encoding='passthrough')
            #print(depths.shape)
            depths = depths[y-bbox.height/2:y+bbox.height/2, x-bbox.width/2:x+bbox.width/2]
            #print(depths.shape)
            depths = [i for i in depths.flatten() if not numpy.isnan(i)]
            #print(len(depths))
            Z = round(float(numpy.median(depths)), 3)
            pose = Pose()
            pose.position.x = round(float((x - c_x) * Z / fx), 3)
            pose.position.y = round(float((y - c_y) * Z / fy), 3)
            pose.position.z = Z
            pub.publish(pose)
        except Exception as e:
            print(e.message)

    else:
        print("not l objects")


if __name__ == '__main__':
    rospy.init_node('xyz')

    tss = ApproximateTimeSynchronizer([Subscriber('/zed_node/left/objects', ObjectArray), 
				                       Subscriber('/zed_node/depth/depth_registered', Image)], 
                                       queue_size=10, 
                                       slop=1)
    tss.registerCallback(callback)
    
    pub = rospy.Publisher('/zed_node/button_Pose_from_Dsensor', Pose, queue_size=10)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        rate.sleep()
