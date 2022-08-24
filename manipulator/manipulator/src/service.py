#!/usr/bin/env python3
import rospy
from manipulator.srv import GetButtons
from manipulator.srv import PressButton, PressButtonResponse 
import numpy as np
from geometry_msgs.msg import Pose
from button import Button
import rtde_control
import rtde_receive
from press_realsense_rtde import connect_ur5, fold
import time
import actionlib
from robotiq_2f_gripper_msgs.msg import CommandRobotiqGripperFeedback, CommandRobotiqGripperResult, CommandRobotiqGripperAction, CommandRobotiqGripperGoal
from robotiq_2f_gripper_control.robotiq_2f_gripper_driver import Robotiq2FingerGripperDriver as Robotiq


    
rtde_c, rtde_r = None, None

def move_to_operate_point(rtde_c, rtde_r, z, base_joint):
    actual_q = [1.5134, -2.2167, 1.6322, -2.5581, -1.4764, 0.0247]
    actual_q[0] = base_joint
    rtde_c.moveJ(actual_q, 0.5, 0.2)
    actual_cart=  rtde_r.getActualTCPPose()
    actual_cart[2] = z
    rtde_c.moveL(actual_cart, 0.1, 0.1)


def connect_ur5(IP = "192.168.131.40"):
    global rtde_c, rtde_r
    ur_connected = False
    try:
        rtde_c = rtde_control.RTDEControlInterface(IP, 
            rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
        rtde_r = rtde_receive.RTDEReceiveInterface(IP)
        ur_connected = True
    except:
        rospy.logfatal("rtde cant connect")
        exit()

        


def press_with_force(target, timeout = 5):
    rtde_c.moveL(target, 0.1, 0.01, True)
    t_start = time.time()
    while time.time() - t_start < timeout:
        if np.linalg.norm(np.array(rtde_r.getActualTCPForce())) > 130: 
            rtde_c.stopL(1)
            break    
    
def detect_and_press(zmin, zmax, base_joint, end_operate, button, rec_deep =0 ):
    
    if(base_joint < 0):
        rtde_c.moveJ([-0.9959610144244593, -1.3471043745623987, 0.8740167617797852, -2.755754295979635, -0.5473888556109827, 0.10433466732501984], 0.1, 0.1)
        #rtde_c.moveJ( [-0.9960568586932581, -1.3802288214312952, 0.9918828010559082, -2.840628449116842, -0.5478561560260218, 0.10456236451864243], 0.1, 0.1)
    time.sleep(10)
    button.update_button()
    button_pose = button.get_button()
    pose = Pose()
    if button_pose is None:
        print("no objects")
        print("-----")
        if end_operate:
            fold(rtde_c, rtde_r, False)
        return PressButtonResponse(False, pose)
    if zmin > button_pose[2] or zmax < button_pose[2]:
        print("button not in range ({0}, {1})".format(zmin, zmax))
        print("-----")
        if req.end_operate:
            fold(rtde_c, rtde_r, False)
        return PressButtonResponse(False, pose)
    print(button.normal)
    print("from", rtde_r.getActualTCPPose())
    print("to  ", button.get_point_with_offset())
    if  abs(button.normal[0]) > 0.25:
        
        #rtde_c.moveL(button.get_point_with_offset(0.20), 0.1, 0.01)
        
        return detect_and_press(zmin, zmax, base_joint, end_operate, button, rec_deep+1)
         
    

    start_press_pose = rtde_r.getActualQ()
    rtde_c.moveL(button.get_point_with_offset(), 0.1, 0.01)
    press_with_force(button.get_button())
    rtde_c.moveL(button.get_point_with_offset(), 0.1, 0.1)
    rtde_c.moveJ(start_press_pose, 0.1, 0.1)

    #rtde_c.moveL(button_pose, 0.1, 0.1)
    move_to_operate_point(rtde_c, rtde_r, (zmax+zmin)/2.0, base_joint)
    if end_operate:
        fold(rtde_c, rtde_r, False)
    
    print("-----")
    pose.position.x = button_pose[0]
    pose.position.y = button_pose[1]
    pose.position.z = button_pose[2]
    pose.orientation.x = button_pose[3]
    pose.orientation.y = button_pose[4]
    pose.orientation.z = button_pose[5]
    return PressButtonResponse(True, pose)
    
def press_button_handler(req):
    robotiq_client = actionlib.SimpleActionClient('command_robotiq_action', CommandRobotiqGripperAction)
    robotiq_client.wait_for_server()
    rospy.loginfo("Connected to the gripper server")
    Robotiq.goto(robotiq_client, pos=0.0, speed=0.1, force=0, block=False)
    zmin = req.zmin
    zmax = req.zmax
    base_joint = req.base_joint
    button = Button(rtde_r)
    move_to_operate_point(rtde_c, rtde_r, (zmax+zmin)/2.0, base_joint)
    rtde_c.teachMode()
    #input("press enter to continue")
    rtde_c.endTeachMode()
    return detect_and_press(zmin, zmax, base_joint, req.end_operate, button)


def press_button_server():
    connect_ur5()
    rospy.init_node("press_button_server", anonymous=True, disable_signals=True)
    s = rospy.Service('press_button', PressButton, press_button_handler)
    print("press button server started")
    rospy.spin()
    pass


if __name__ == "__main__":
    press_button_server()
