#!/usr/bin/env python
import math
import os
import rospy
from geometry_msgs.msg import PoseArray
from manipulator.srv import PressButton, PressButtonResponse
import time
from rospy.exceptions import ROSException 
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from dh4 import dh4
from husky_ctrl import husky_control

class GlobVars:
    trajectory = None
    ctrl_stat= None
    press_button = None
    pub_traject = None
    pub_node_killer = None
    pub_goal = None
    sub_traject = None
    sub_ctrl_status= None
    wait_for_ctrl = False
    
    can_move = False
    press_now = False
    reach_press = True
    gamepad = dh4()

    


class ctrl_msg:
    reached = "reached"
    not_reached = "not reached"

def x_trigger():
    GlobVars.reach_press = not  GlobVars.reach_press
    if GlobVars.reach_press:
        rospy.loginfo("MODE  ON: press button when reached target")
    else:
        rospy.loginfo("MODE OFF: press button when reached target")
def o_trigger():
    GlobVars.press_now = not GlobVars.press_now
    if GlobVars.press_now:
        rospy.loginfo("MODE  ON: press button now")
    else:
        rospy.loginfo("MODE OFF: press button now")

def sqr_trigger():
    GlobVars.can_move = not GlobVars.can_move
    if GlobVars.can_move:
        rospy.loginfo("MODE  ON: move when recieve trajectory")
    else:
        rospy.loginfo("MODE OFF: move when recieve trajectory") 
  

def trajectory_callback(pose_array):
    #global trajectory
    if len(pose_array.poses) > 0: 
        rospy.loginfo("Get tragectory")
        GlobVars.trajectory = pose_array
        
    else:
        rospy.logwarn("Empty tragectory")
    GlobVars.pub_traject.publish(pose_array)

    

def status_callback(stat):
    rospy.loginfo(stat)
    #global ctrl_stat
    
    GlobVars.ctrl_stat = str(stat.data)
    if stat.data == ctrl_msg.reached:
        GlobVars.wait_for_ctrl = False
        rospy.loginfo("controller reached")

def kill_lol():
    os.system("rosnode kill /newcontrol")
    GlobVars.pub_node_killer.publish("kill lol rtab")
    
def kill_solo():
    print("trying to kill solo")
    GlobVars.pub_node_killer.publish("kill solo")
    
    

def run_solo():
    time.sleep(4)
    kill_lol()
    time.sleep(4)
    GlobVars.pub_node_killer.publish("run solo")
    time.sleep(30)
    
    


def loop_test_pipeline():
    if GlobVars.can_move:
        goal_point_msg = PoseStamped()
        goal_point_msg.pose.position.x =  -30.3
        goal_point_msg.pose.position.y =  -12.45
        goal_point_msg.pose.position.z =  0
        goal_point_msg.pose.orientation.x =  0
        goal_point_msg.pose.orientation.y =  0
        goal_point_msg.pose.orientation.z =  0.73
        goal_point_msg.pose.orientation.w =  0.67
        goal_point_msg.header.frame_id = "map"
        goal_point_msg.header.stamp = rospy.get_rostime()
        GlobVars.pub_goal.publish(goal_point_msg)
        GlobVars.can_move = False
    
    if not GlobVars.trajectory is None :
        GlobVars.ctrl_stat =None
        rospy.loginfo("WARNING! Start moving")
        
        GlobVars.trajectory = None
        GlobVars.wait_for_ctrl = True

    if GlobVars.ctrl_stat == ctrl_msg.reached and not GlobVars.wait_for_ctrl:
        
        GlobVars.ctrl_stat = None
        #husky_control.ride_meters(0.5, 0.1)
        rospy.loginfo("THIS SECTION2")
        run_solo()
        GlobVars.press_button(0, 1.5, 1.5134, True)
        GlobVars.can_move  = False
        husky_control.rotate_base(-math.pi/2)
        husky_control.ride_meters(1.85, 0.2)
        husky_control.rotate_base(math.pi/2)
        husky_control.ride_meters(2.35, 0.3)
        GlobVars.press_button(0, 1.6, -0.5, True)
        GlobVars.can_move = False
        kill_solo()
    
    
    


def loop():
    #global trajectory, ctrl_stat, press_now
    
    if (not GlobVars.trajectory is None) and GlobVars.can_move:
            rospy.loginfo("WARNING! Start moving")
            GlobVars.pub_traject.publish(GlobVars.trajectory)
            GlobVars.ctrl_stat =None
            GlobVars.trajectory = None
    if GlobVars.ctrl_stat == ctrl_msg.not_reached:
        return
    
    if (GlobVars.ctrl_stat == ctrl_msg.reached and GlobVars.trajectory is None and GlobVars.reach_press) or GlobVars.press_now:
        #killing rtabmap and other and run solo
        GlobVars.pub_node_killer.publish("solo") 
        time.sleep(10)

        rospy.logdebug("Calling service")
        res = service_call(0, 1.4, 1.5134, True)
        
        if res.result:
            rospy.loginfo("pressed")
        else:
            rospy.loginfo("no target")
        GlobVars.ctrl_stat = None
        GlobVars.press_now=False

        #killing solo and run rtabmap and other 
        GlobVars.pub_node_killer.publish("lol")
        time.sleep(10)
    

def service_call(*args):
    # try:
    #     press_button.wait_for_service(timeout=1)
    # except ROSException:
    #     return None
    return GlobVars.press_button(*args)
       
    

def main():
    #global sub_traject, G, press_button, sub_ctrl_status
    GlobVars.gamepad.bind_trigger(dh4.x_btn, x_trigger)
    GlobVars.gamepad.bind_trigger(dh4.crcl_btn, o_trigger)
    GlobVars.gamepad.bind_trigger(dh4.sqr_btn, sqr_trigger)

    try:
        node_name           =rospy.get_param("node_name")
        path_topic          =rospy.get_param("path_topic")
        send_path_to_topic  =rospy.get_param("send_path_to_topic")
        pres_srv_name       =rospy.get_param("press_srv_name")
        ctrl_status_topic   =rospy.get_param("status_topic")
    except KeyError as e:
        rospy.logwarn(e)
        rospy.logwarn("Tactical node start with default params")
        node_name           ="tactical_node"
        path_topic          ="trajectory"
        send_path_to_topic  ="trajectory_from_proxy"
        pres_srv_name       ="press_button"
        ctrl_status_topic   ="control_status"
    rospy.init_node(node_name)
    GlobVars.sub_traject = rospy.Subscriber(path_topic, PoseArray, callback=trajectory_callback)
    GlobVars.pub_traject = rospy.Publisher(send_path_to_topic, PoseArray, queue_size=1)
    #GlobVars.pub_traject = rospy.Publisher(send_path_to_topic, PoseArray, queue_size=10)
    GlobVars.pub_goal    = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)
    GlobVars.press_button = rospy.ServiceProxy(pres_srv_name, PressButton)
    GlobVars.sub_ctrl_status = rospy.Subscriber(ctrl_status_topic, String, callback=status_callback)
    GlobVars.pub_node_killer = rospy.Publisher("/do", String, queue_size=1)
    rospy.loginfo("Wait for press service")
    time.sleep(1)
    kill_solo()
    husky_control.connect_husky()

    try:
        GlobVars.press_button.wait_for_service(1)
    except ROSException:
        rospy.logwarn("Press service connection timeout. Will be working like proxy for trajectory and trying to connect.")
    while not rospy.is_shutdown():
        #loop()
        loop_test_pipeline()
    
    


if __name__ == "__main__":
    dh4
    main()