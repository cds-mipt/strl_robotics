#!/usr/bin/python
import os
import rospy
import numpy
from message_filters import ApproximateTimeSynchronizer, Subscriber
import math
import tf

from geometry_msgs.msg import Pose, PoseStamped, Point
from std_msgs.msg import Float32
from sensor_msgs.msg import Image
from std_msgs.msg import Header
from camera_objects_msgs.msg import ObjectArray, Object, RLE

from cv_bridge import CvBridge
from tf.transformations import quaternion_from_euler, unit_vector, projection_matrix, quaternion_from_matrix, euler_from_quaternion
from visualization_msgs.msg import Marker, MarkerArray
from button_srv.srv import GetButtons, GetButtonsResponse
from scipy.spatial.transform import Rotation


rospy.init_node("xyz")

bridge = CvBridge()

fx = 683.7217407226562
fx_old=682.7612915039062
fy = 683.7217407226562
fy_old=682.7612915039062

c_x = 615.113525390625
c_x_old = 615.1388549804688
c_y = 345.3251647949219
c_y_old=345.3089599609375

pose_array = []

def produce_center_point(roi):
    #simetric object
    return int(roi.x_offset + roi.width / 2), int(roi.y_offset + roi.height / 2)

def to_manipulator(pose):
    #to base_link
    #pose.header.frame_id = "zed_left_camera_frame"
    #pose = tf_listener.transformPose("base_link", pose)
    
    lc = pose.position
    lc_coord = numpy.array([[lc.x], [lc.y], [lc.z]])
    T = [-0.239, 0.06, 0.75]
    #T = [-0.4088, 0.075, 0.76]
    R_cl2b = numpy.array([[0, 0, 1],
                          [-1, 0, 0],
                          [0, -1, 0]])
    mp_coord = numpy.matmul(R_cl2b, lc_coord)
    pose.position.x = mp_coord[0][0] + T[0]
    pose.position.y = mp_coord[1][0] + T[1]
    pose.position.z = mp_coord[2][0] + T[2]
    return pose

def get_z_with_neighborhoods(x ,y, depths, bbox, area):
    depths = depths[y-area[1]:y+area[1],x-area[0]:x+area[0]]
    depths = [i for i in depths.flatten() if not numpy.isnan(i)]
    return round(float(numpy.median(depths)), 3)

def print_pose(p):
    print p.position.x, p.position.y, p.position.z

def pose_in_camL_frame(x, y, depths, bbox, area):
    pose = Pose()
    Z = get_z_with_neighborhoods(x, y, depths, bbox, area)
    #print("Depth", Z)
    pose.position.x = round(float((x - c_x) * Z / fx), 3)
    pose.position.y = round(float((y - c_y) * Z / fy), 3)
    pose.position.z = Z
    return pose

def __sub__(self, other):
    return Pose(position=Point(self.position.x - other.position.x, self.position.y - other.position.y, self.position.z - other.position.z))

def normalize(self):
    point = self.position
    l = math.sqrt(point.x**2 + point.y**2 + point.z**2)
    a = point.x/l
    b = point.y/l
    c = point.z/l
    return Pose(position=Point(a, b, c))

def as_numpy(self):
    return numpy.array([self.position.x, self.position.y, self.position.x])

setattr(Pose, '__sub__', __sub__)
setattr(Pose, 'as_numpy', as_numpy)
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

def produce_marker(r, g, b, pose, id):
   
    marker = Marker()
    marker.header.frame_id = "base_link"
    marker.type = 2
    marker.action = 0
    marker.id = id

    marker.scale.x = 0.025
    marker.scale.y = 0.025
    marker.scale.z = 0.025
    marker.color.a = 1.0
    marker.color.r = r
    marker.color.g = g
    marker.color.b = b
    marker.pose.orientation.w = 1.0

    marker.pose = pose
            

    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    return marker

def callback(data_l, depth_l):
    if len(data_l.objects) == 1:
        bbox = data_l.objects[0].bbox
        print('---')
        print("Number of objects-", len(data_l.objects))
        x, y = produce_center_point(bbox)

        try:
            markerarray = MarkerArray()
            depths = bridge.imgmsg_to_cv2(depth_l, desired_encoding='passthrough')

            pose_st = PoseStamped()
           # print("1")
            #coordinates in CamL frame
            pose = pose_in_camL_frame(x, y, depths, bbox, [int(bbox.height*1.2), int(bbox.width*1.2)]) #zamenit na triangulaziu

            #coordinates in Base frame
            pose = to_manipulator(pose)
            #print("Center")
            #print_pose(pose)

            #get points from button base surface in Base frame and in range 2x button size with the assumption 
            # that base surface is paralleled to the button
            # ! pod ostrimi uglami x - bbox.width budut hvatat tochki mimo
            pose_l_up = to_manipulator(pose_in_camL_frame(int(x-0.75*bbox.width), y+bbox.height, depths, bbox, [3,3]))
            pose_r_up = to_manipulator(pose_in_camL_frame(int(x+0.75*bbox.width), y+bbox.height, depths, bbox, [3,3]))

            
            if any([numpy.isnan(pose_l_up.position.x), numpy.isnan(pose_l_up.position.y), numpy.isnan(pose_l_up.position.z),
                    numpy.isnan(pose_r_up.position.x), numpy.isnan(pose_r_up.position.y), numpy.isnan(pose_r_up.position.z)]):
                return

            pose_l_up.position.z = pose_r_up.position.z
            

            marker_center = produce_marker(0,0,1,pose, 0)
            marker_left = produce_marker(1,0,1, pose_l_up, 1)
            marker_right = produce_marker(1,0,1,pose_r_up, 2)
            markerarray.markers = []
            markerarray.markers.extend([marker_center, marker_left, marker_right])
            #markerarray.markers.append(marker_left)
            print("Markers count", len(markerarray.markers))
            marker_pub.publish(markerarray)

            #plane is always must be perpendicular to the floor
            #pose_up = to_manipulator(pose_in_camL_frame(x, int(y-bbox.height), depths, bbox))
            #pose_up.position.x = pose.position.x
            #pose_up.position.y = pose.position.y
            #pose_up = pose_up - pose
            #pose_up = pose_up.normalize()

            #a, b, c = numpy.cross([pose_up.position.x, pose_up.position.y, pose_up.position.z],
                 #                                    [pose_r.position.x, pose_r.position.y, pose_r.position.z])
            
            #pose_n = Pose(position=Point(a, b, c))
            #pose_n = pose_n.normalize()
            
            #if any([numpy.isnan(i) for i in numpy.concatenate((pose.as_numpy(), pose_r.as_numpy(), pose_up.as_numpy(), pose_n.as_numpy()))]):
             #   return


            #print("right", pose_r.position.x, pose_r.position.y, pose_r.position.z)
            #print("norm", pose_n.position.x, pose_n.position.y, pose_n.position.z)
            #print("up", pose_up.position.x, pose_up.position.y, pose_up.position.z)

            
            #print(math.degrees(math.acos(a)), math.degrees(math.acos(b)), math.degrees(math.acos(c)))
            #quaternion to button
            #Q = quaternion_from_euler(math.acos(a),math.acos(b),math.acos(c), 'rxyz')
            #Q = quaternion_from_matrix(numpy.array([[pose_r.position.x, pose_r.position.y, pose_r.position.z, 0], 
            pose_l_up = pose_r_up - pose_l_up
            pose_l_up = pose_l_up.normalize()
            pose_l_up.position.y = abs(pose_l_up.position.y)

            angle = round(math.degrees(math.atan(pose_l_up.position.x/pose_l_up.position.y)))

            
            print("Angle", angle)
            angle = math.radians(angle)
            #angle_pub.publish(angle)

            #Q = quaternion_from_matrix(numpy.array([[math.cos(math.radians(angle)), -math.sin(math.radians(angle)), 0, 0], 
            #                                        [math.sin(math.radians(angle)), math.cos(math.radians(angle)), 0, 0], 
            #                                        [0, 0, 1, 0], 
            #                                        [0, 0, 0, 1]]))
            #pose.orientation.x = Q[0]
            #pose.orientation.y = Q[1]
            #pose.orientation.z = Q[2]
            #pose.orientation.w = Q[3]

            #print("Quant", Q[0], Q[1], Q[2], Q[3])

            pose_st.header.frame_id = "base_link"
            pose_st.header.stamp = rospy.Time.now()
            pose_st.pose = pose
            print("In base_link")
            print_pose(pose)
            
            pose_st = tf_listener.transformPose("ur_arm_base", pose_st)
            
            print("Center")
            print_pose(pose_st.pose)
            
            r = Rotation.from_euler('xyz', [-1.57, 0, -(3.14-angle)])
            r = r.as_rotvec()
            print("Rotvec", r)


            pose_st.pose.position.y += 0.07 #0.03 + 0.02 for Kostya push action/ or 0.05 + 0.02 for red button
            pose_st.pose.position.z += 0.04
            
            #print_pose(pose_st)
            
            #r = r.as_rotvec()
            #pose_st.pose.orientation.x = r[0]
            #pose_st.pose.orientation.y = r[1]
            #pose_st.pose.orientation.z = r[2]
            #pose_st.pose.orientation.w = 1
            #print(r)
            #print(euler)

            if numpy.isnan(pose_st.pose.position.x):
                print("skip")
                return

            #pub.publish(pose_st)

            if len(pose_array) >= 10:
                global pose_array
                pose_array.pop(0)
            pose_array.append(pose_st.pose)
        except Exception as e:
            print(e.message)

    elif not data_l.objects:
        print("not l objects")


def handle_get_buttons(req):
    d = {}
    pose_f = pose_array[0]
    for pose in pose_array:
        key = ";".join([str(round(pose.position.x,2)), str(round(pose.position.y,2)), str(round(pose.position.z, 2))])
        if not key in d:
            d[key] = 1
        else:
            d[key] += 1
    d = sorted(d.items(), key=lambda x:x[1])
    pose, pose_encounter = d[-1]
    final_pose_array = [float(i) for i in pose.split(";")]
    print(final_pose_array)
    pose_f.position.x = final_pose_array[0]
    pose_f.position.y = final_pose_array[1]
    pose_f.position.z = final_pose_array[2]
    print("Send pose with", pose_encounter, "encounter")
    print(d)
    global pose_array
    pose_array = []
    #os.system("rosnode kill /zed_node/left/solo_node_left")
    return GetButtonsResponse(pose_f)


if __name__ == '__main__':
    

    tss = ApproximateTimeSynchronizer([Subscriber('/zed_node/left/objects', ObjectArray), 
				                       Subscriber('/zed_node/depth/depth_registered', Image)], 
                                       queue_size=30, 
                                       slop=2)
    tss.registerCallback(callback)
    s = rospy.Service('get_buttons_pose', GetButtons, handle_get_buttons)
    pub = rospy.Publisher('/zed_node/button_Pose_from_Dsensor', PoseStamped, queue_size=10)
    marker_pub = rospy.Publisher('/marker', MarkerArray, queue_size=1)
    angle_pub = rospy.Publisher('/button_angle', Float32, queue_size=1)
    rate = rospy.Rate(6)
    tf_listener = tf.TransformListener()

    while not rospy.is_shutdown():
        rate.sleep()
