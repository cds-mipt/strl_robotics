import rtde_receive
import math
import numpy as np
from scipy.spatial.transform import Rotation
from manipulator.srv import GetButtons
import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header 
import time

class Button:
    def __init__(self, rtde_r):
        self.pub = rospy.Publisher('button_pose', PoseStamped, queue_size=5)
        self.rtde_r = rtde_r
        self.id = 0
        # self.l = 0.18
        # self.h = 0.08
        # self.k = 0.03
        # self.k = 0.03
        # self.h = 0.08
        # self.l = 0.12
        self.k = 0.03
        self.h = 0.08
        self.l = 0.12
        self.button = None
        self.normal = None
        self.cart_on_update = None
        self.button_rel_cam = None
        self.btn_topic = 'get_buttons_pose'

    def update_button(self):
        print("wait for service")
        rospy.wait_for_service(self.btn_topic)
        try:
            get_buttons_pose = rospy.ServiceProxy(self.btn_topic, GetButtons)
            rospy.loginfo(1)
            pose = get_buttons_pose(1)
            rospy.loginfo(pose.pose)
            if pose.pose==None :
                return False
            
            self.cart_on_update = self.rtde_r.getActualTCPPose()
            cords, self.normal = self.pose_to_cords_normal(pose.pose)
            self.button = self._to_manipulator(cords)
            self.button_rel_cam = cords
            pose_to_pub = PoseStamped()
            pose_to_pub.pose.position.x = self.button[0]
            pose_to_pub.pose.position.y = self.button[1]
            pose_to_pub.pose.position.z = self.button[2]
            oqart = Rotation.from_euler("xyz",
                    self.button[3:]).as_quat()
            pose_to_pub.pose.orientation.x = oqart[0]
            pose_to_pub.pose.orientation.y = oqart[1]
            pose_to_pub.pose.orientation.z = oqart[2]
            pose_to_pub.pose.orientation.w = oqart[3]
            
            self.pub.publish(pose_to_pub)
            
            return True
        except rospy.ServiceException as e:
            return False


    def _to_manipulator(self, cords, l=None, h= None, k=None):
        if l == None:
            l = self.l
        if h == None:
            h = self.h
        if k == None:
            k = self.k

        reals_to_tool = np.array([[1, 0, 0, -k], 
                                [0, 1, 0, -h],
                                [0, 0, 1, -l],
                                [0, 0, 0, 1]])
        extra_len_p = np.concatenate([cords, [1]])
        point_rel_tool = np.matmul(reals_to_tool, extra_len_p)[0:3]
        rot_mat = Rotation.from_rotvec(self.cart_on_update[3:]).as_matrix()
        trans_to_base = np.array([[1, 0, 0, self.cart_on_update[0]], 
                                [0, 1, 0, self.cart_on_update[1]],
                                [0, 0, 1, self.cart_on_update[2]],
                                [0, 0, 0, 1]])
        loc_button = np.matmul(trans_to_base, np.concatenate([np.matmul(rot_mat, point_rel_tool), [1]]))
        strange_rotation = Rotation.from_euler("xyz", [ math.atan2( self.normal[1], self.normal[2]), -math.atan2( self.normal[0], self.normal[2]), 0]).as_matrix()
        #strange_rotation = Rotation.from_euler("xyz", [ math.atan2( self.normal[1], self.normal[2]), -math.atan2( self.normal[0], self.normal[2]), 0]).as_matrix()
        strange_rotation = np.matmul(rot_mat, np.linalg.inv(strange_rotation))
        strange_rotation = Rotation.from_matrix(strange_rotation).as_rotvec()
        
        button_location_l = np.array( [loc_button[0], loc_button[1], loc_button[2],strange_rotation[0],  strange_rotation[1],  strange_rotation[2]])
        
        return button_location_l

    def get_button(self):
        if self.button is None:
            if not self.update_button():
                return None
        return self.button
    
    def pose_to_cords_normal(self, pose):
        return np.array([pose.position.x, pose.position.y, pose.position.z]), np.array([pose.orientation.x, pose.orientation.y, pose.orientation.z])

    def get_point_with_offset(self, offset = 0.1):
        return self._to_manipulator(self.button_rel_cam - offset* self.normal)
        buff = self.button.copy()
        print(self.normal)
        buff[0:3] -= offset*self.normal
        return buff
