from os import EX_NOHOST
import rospy 
from geometry_msgs.msg import Twist
import time
import math
class husky_control:
    @staticmethod
    def connect_husky():
        husky_control.cmd_vel_topic = rospy.Publisher("/husky_velocity_controller/cmd_vel", Twist, queue_size =10)
    @staticmethod
    def move(a_cmd):
        t = Twist()
        t.linear.x  = a_cmd[0]
        t.linear.y  = a_cmd[1]
        t.linear.z  = a_cmd[2]
        t.angular.x = a_cmd[3]
        t.angular.y = a_cmd[4]
        t.angular.z = a_cmd[5]
        husky_control.cmd_vel_topic.publish(t)
    @staticmethod
    def ride_meters(len, speed = 0.01):
        if len < 0:
            cmd = [-speed, 0, 0, 0, 0, 0]
        else:
            cmd = [speed, 0, 0, 0, 0, 0]
        t_cmd = rospy.Time().now()
        while rospy.Time().now() - t_cmd < rospy.Duration(math.fabs(len/speed)):
            husky_control.move(cmd)
        husky_control.move([0, 0, 0, 0, 0, 0])

    @staticmethod
    def rotate_base(angle, speed = math.pi/15):
        if angle < 0:
            cmd = [0, 0, 0, 0, 0, -speed]
        else:
            cmd = [0, 0, 0, 0, 0, speed]
        t_cmd = rospy.Time().now()
        print(angle/speed)
        while rospy.Time().now() - t_cmd < rospy.Duration(math.fabs(angle/speed)):
            husky_control.move(cmd)
        husky_control.move([0, 0, 0, 0, 0, 0])