#!/usr/bin/python

import rospy
import numpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import math

from geometry_msgs.msg import Pose, PoseStamped, Point
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from camera_objects_msgs.msg import ObjectArray, Object, RLE

from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler, unit_vector, projection_matrix, quaternion_from_matrix
from visualization_msgs.msg import Marker

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

def get_z_with_neighborhoods(x ,y, depths, bbox):
    depths = depths[y-bbox.height/2:y+bbox.height/2, x-bbox.width/2:x+bbox.width/2]
    depths = [i for i in depths.flatten() if not numpy.isnan(i)]
    return round(float(numpy.median(depths)), 3)

def plane_equation(pose1, pose2, pose3):
    print('!!!')
    print('a', pose1.position.x, pose1.position.y, pose1.position.z)
    print('b', pose2.position.x, pose2.position.y, pose2.position.z)
    print('c', pose3.position.x, pose3.position.y, pose3.position.z)

    a1 = pose2.position.x - pose1.position.x
    b1 = pose2.position.y - pose1.position.y
    c1 = pose2.position.z - pose1.position.z
    a2 = pose3.position.x - pose1.position.x
    b2 = pose3.position.y - pose1.position.y
    c2 = pose3.position.z - pose1.position.z

    a = b1 * c2 - b2 * c1
    b = a2 * c1 - a1 * c2
    c = a1 * b2 - b1 * a2
    d = (- a * pose1.position.x - b * pose1.position.y - c * pose1.position.z)

    x = numpy.array([a, b, c])
    a, b, c = x / numpy.linalg.norm(x)
    return a, b, c

def print_pose(pose):
    print pose.position.x, pose.position.y, pose.position.z

def pose_in_camL_frame(x, y, depths, bbox):
    pose = Pose()
    Z = get_z_with_neighborhoods(x, y, depths, bbox)
    pose.position.x = round(float((x - c_x) * Z / fx), 3)
    pose.position.y = round(float((y - c_y) * Z / fy), 3)
    pose.position.z = Z
    return pose

def __sub__(self, other):
    return Pose(position=Point(self.position.x - other.position.x, self.position.y - other.position.y, self.position.z - other.position.z))

def normalize(self):
    point = self.position
    x = numpy.array([point.x, point.y, point.z])
    a, b, c = x / numpy.linalg.norm(x)
    return Pose(position=Point(a, b, c))

setattr(Pose, '__sub__', __sub__)
setattr(Pose, 'normalize', normalize)

def make_surface(main_pose, pose_list):
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = 11
    marker.action = 0

    marker.scale.x = 30
    marker.scale.y = 30
    marker.scale.z = 30
    marker.color.a = 1.0
    marker.color.r = 1.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.pose.orientation.w = 1.0

    marker.pose.position = main_pose.position
    marker.points = [Point(0, 0 ,0),
                     main_pose.position - pose_list[0].position,
                     main_pose.position - pose_list[1].position]
    marker_pub.publish(marker)

def callback(data_l, depth_l):
    if data_l.objects:
        bbox = data_l.objects[0].bbox
        x, y = produce_center_point(bbox)

        try:
            depths = bridge.imgmsg_to_cv2(depth_l, desired_encoding='passthrough')

            pose_st = PoseStamped()

            #coordinates in CamL frame
            pose = pose_in_camL_frame(x, y, depths, bbox)

            #coordinates in Base frame
            pose = to_manipulator(pose)

            #get points from button base surface in Base frame and in range 2x button size with the assumption 
            # that base surface is paralleled to the button
            pose_r = to_manipulator(pose_in_camL_frame(int(x+2*bbox.width), y, depths, bbox))
            pose_r.position.z = pose.position.z
            pose_r = pose_r - pose
            pose_r = pose_r.normalize()

            #plane is always must be perpendicular to the floor
            pose_up = to_manipulator(pose_in_camL_frame(x, int(y-2*bbox.height), depths, bbox))
            pose_up.position.x = pose.position.x
            pose_up.position.y = pose.position.y
            pose_up = pose_up - pose
            pose_up = pose_up.normalize()

            a, b, c = numpy.cross([pose_up.position.x, pose_up.position.y, pose_up.position.z],
                                    [pose_r.position.x, pose_r.position.y, pose_r.position.z])
            
            pose_n = Pose(position=Point(a, b, c))
            pose_n = pose_n.normalize()
            
            if any([numpy.isnan(i) for i in numpy.concatenate((pose.as_numpy(), pose_r.as_numpy(), pose_up.as_numpy(), pose_n.as_numpy()))]):
                return


            #print("right", pose_r.position.x, pose_r.position.y, pose_r.position.z)
            #print("norm", pose_n.position.x, pose_n.position.y, pose_n.position.z)
            #print("up", pose_up.position.x, pose_up.position.y, pose_up.position.z)

            #draw new basis
            marker = Marker()
            marker.header.frame_id = "base_link"
            marker.type = 5
            marker.action = 0

            marker.scale.x = 0.01
            marker.scale.y = 1
            marker.scale.z = 1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.pose.orientation.w = 1.0

            marker.pose.position = Point(0, 0, 0)
            marker.points = [Point(0, 0, 0), pose_r.position,
                             Point(0, 0, 0), pose_n.position,
                             Point(0, 0, 0), pose_up.position]
            marker_pub2.publish(marker)


            #print(math.degrees(math.acos(a)), math.degrees(math.acos(b)), math.degrees(math.acos(c)))
            #quaternion to button
            #Q = quaternion_from_euler(math.acos(a),math.acos(b),math.acos(c), 'rxyz')
            #Q = quaternion_from_matrix(numpy.array([[pose_r.position.x, pose_r.position.y, pose_r.position.z, 0], 
            #                                        [pose_n.position.x, pose_n.position.y, pose_n.position.z, 0], 
            #                                        [pose_up.position.x, pose_up.position.y, pose_up.position.z, 0], 
            #                                        [0, 0, 0, 1]]))
    
            #Q = quaternion_from_matrix(numpy.array([[0.71, 0.71, 0, 0], 
            #                                        [-0.71, 0.71, 0, 0], 
            #                                        [0, 0, 1, 0], 
            #                                        [0, 0, 0, 1]]))
            angel_x = int(math.degrees(math.acos(pose_r.position.x)))
            print(angel_x)
            Q = quaternion_from_matrix(numpy.array([[math.cos(math.radians(-angel_x)), -math.sin(math.radians(-angel_x)), 0, 0], 
                                                    [math.sin(math.radians(-angel_x)), math.cos(math.radians(-angel_x)), 0, 0], 
                                                    [0, 0, 1, 0], 
                                                    [0, 0, 0, 1]]))
            pose.orientation.x = Q[0]
            pose.orientation.y = Q[1]
            pose.orientation.z = Q[2]
            pose.orientation.w = Q[3]

            pose_st.header.frame_id = "base_link"
            pose_st.header.stamp = rospy.Time.now()
            pose_st.pose = pose
            pub.publish(pose_st)
            
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
    
    pub = rospy.Publisher('/zed_node/button_Pose_from_Dsensor', PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher('/surface', Marker, queue_size=1)
    marker_pub2 = rospy.Publisher('/normal', Marker, queue_size=1)
    rate = rospy.Rate(3)

    while not rospy.is_shutdown():
        rate.sleep()
