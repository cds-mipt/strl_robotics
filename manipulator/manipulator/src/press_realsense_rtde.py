#!/usr/bin/env python3
import rtde_control
import rtde_receive
import time
import datetime
import csv
from std_msgs.msg import String
from copy import deepcopy
from manipulator.srv import GetButtons
import rospy
import numpy as np
from scipy.spatial.transform import Rotation
import math
from button import *
UR_IP = "192.168.131.40"

def pipe_line(rtde_c, rtde_r):
    start_q = rtde_r.getActualQ()
    rtde_c.moveJ([start_q[0], -2.2167, 1.6322, -2.5581, -1.4764, 0.0247], 0.1, 0.01)
    move_to_operate_point(rtde_c, rtde_r, False)
    time.sleep(3)
    pose  = None
    
    
    while True:
        start_q = rtde_r.getActualQ()
        while pose == None:
            start_q[0]-=math.pi/2
            rtde_c.moveJ(start_q, 0.3, 0.1)
            time.sleep(5)
            while pose != None:
                pose = get_button()
            time.sleep(7)
            pose  = get_button()
        actual_q = rtde_r.getActualQ()
        actual_cart = rtde_r.getActualTCPPose()
        button_location_l = to_manipulator(pose,actual_cart)
        if abs(button_location_l[0]) >0.2 and abs(button_location_l[0]) < 0.8 and abs(button_location_l[1]) < 0.8 and button_location_l[3] != math.nan:
            break
        pose = None
    print( button_location_l)
    in_front = deepcopy(button_location_l)
    #in_front[1] += 0.08
    #in_front[2] += 0.06
    rtde_c.moveL(in_front, 0.1, 0.01)
    #parameters for logging
    t_list = []
    q_list = []
    qd_list = []
    t_list.append(time.time())
    q_list.append(rtde_r.getActualQ())
    qd_list.append(rtde_r.getActualQd()) 

    #parameters for servoing
    lf_list = []
    lf_list.append( rtde_r.getActualTCPForce() )
    dt = 0.024
    lookahead_time = 0.03
    gain = 100
    new_pos = rtde_r.getActualTCPPose()
    new_pos[1] -= 0.001

    # # check servo push
    # t_list.append(time.time())
    # q_list.append(rtde_r.getActualQ())
    # qd_list.append(rtde_r.getActualQd()) 
    # lf_list.append( rtde_r.getActualTCPForce() )
    # for i in range(200):  
    #     start = time.time()
    #     rtde_c.servoL(new_pos, 0.5, 0.25, dt, lookahead_time, gain)
    #     lf_list.append( rtde_r.getActualTCPForce() )
    #     t_list.append(time.time())
    #     q_list.append(rtde_r.getActualQ())
    #     qd_list.append(rtde_r.getActualQd()) 
    #     if lf_list[i][1] > 60:
    #         break
    #     new_pos = rtde_r.getActualTCPPose()
    #     new_pos[1] -= 0.001
    #     end = time.time()
    #     duration = end - start
    #     if duration < dt:
    #         time.sleep(dt - duration)
    #rtde_c.servoStop()
    time.sleep(3)
    # release the button
    #rtde_c.moveL(in_front, 0.1, 0.01)
    #move_to_operate_point(rtde_c, rtde_r, False)
    rtde_c.moveJ(start_q, 0.1, 0.01)
    #fold(rtde_c, rtde_r, False)


def get_button():
    print("wait for service")
    
    rospy.wait_for_service('get_buttons_pose')
    try:
        get_buttons_pose = rospy.ServiceProxy('get_buttons_pose', GetButtons)
        pose = get_buttons_pose(1)
        if pose.pose==None :
            return None
        return pose
    except rospy.ServiceException as e:
        return None
            

def connect_ur5(IP = "192.168.131.40"):
    rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
        rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
    rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
    return rtde_c, rtde_r


def move_to_operate_point(rtde_c, rtde_r, ask = True):
    # move to intermediate
    #intermediate_point_j = [1.3479, -1.9969, 1.5243, -2.6714, -1.3117, 0.0253]
    intermediate_point_j = [1.5134, -2.2167, 1.6322, -2.5581, -1.4764, 0.0247]
    if ask:
        inp = input("Move to intermediate point? y/n: ")[0]
        if (inp == 'y'):    
            rtde_c.moveJ(intermediate_point_j, 0.1, 0.1)
            return True
        else:
            print ("Skipping")
            return False
    else:
        rtde_c.moveJ(intermediate_point_j, 0.1, 0.1)
        return True
    

def to_manipulator(pose, actual_cart, h=0.08, l=0.16, k =0, offset = 0.02):
    target = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
    target_normal = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z])
    
    
    reals_to_tool = np.array([[1, 0, 0, -k], 
                            [0, 1, 0, -h],
                            [0, 0, 1, -l],
                            [0, 0, 0, 1]])
    extra_len_p = np.concatenate([target, [1]])
    point_rel_tool = np.matmul(reals_to_tool, extra_len_p)[0:3]
    #point_rel_tool[2] -= offset
    rot_mat = Rotation.from_rotvec(actual_cart[3:]).as_matrix()
    trans_to_base = np.array([[1, 0, 0, actual_cart[0]], 
                            [0, 1, 0, actual_cart[1]],
                            [0, 0, 1, actual_cart[2]],
                            [0, 0, 0, 1]])
    loc_button = np.matmul(trans_to_base, np.concatenate([np.matmul(rot_mat, point_rel_tool), [1]]))
    normal_rel_base = target_normal
    # normal_rel_base = np.matmul(-reals_to_tool, np.concatenate([target_normal, [1]]))
    # normal_rel_base = normal_rel_base[0:3]
    #normal_rel_base /= np.linalg.norm(normal_rel_base)
    print( "normal",  normal_rel_base)
    strange_rotation = Rotation.from_euler("xyz", [  0, -math.atan2( normal_rel_base[0], normal_rel_base[2]), 0]).as_matrix()
    print([  math.atan2( -normal_rel_base[1], normal_rel_base[2]), math.atan2( normal_rel_base[0], normal_rel_base[2]), 0])
    strange_rotation = np.matmul(rot_mat, np.linalg.inv(strange_rotation))
    strange_rotation = Rotation.from_matrix(strange_rotation).as_rotvec()
    
    button_location_l = [loc_button[0], loc_button[1], loc_button[2],strange_rotation[0],  strange_rotation[1],  strange_rotation[2]]
    
    return button_location_l


def fold(rtde_c, rtde_r,  ask = True):
    if ask:
        inp = input("Fold the robot? y/n: ")[0]
        if (inp == 'y'):
            folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
            rtde_c.moveJ(folded_joints, 0.1, 0.1)
        else:
            print ("Skipping")
    else: 
        folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
        rtde_c.moveJ(folded_joints, 0.4, 0.2)


def  main():
    rospy.init_node("press_button", anonymous=True, disable_signals=True)
    #print(get_button())
    rtde_c, rtde_r = connect_ur5(UR_IP)


    inp = input("Start all? y/n:")
    if inp == 'y':
        pipe_line(rtde_c, rtde_r)
        exit()
    
    fold(rtde_c, rtde_r)
    move_to_operate_point(rtde_c, rtde_r)
    btn = Button(rtde_r)
    btn.update_button()
    button_location_l = btn.get_point_with_offset(0.1)
    # pose  = get_button()
    # actual_q = rtde_r.getActualQ()
    # actual_cart = rtde_r.getActualTCPPose()
    # button_location_l = to_manipulator(pose,actual_cart)
    # print actual cartesian position
    print(rtde_r.getActualTCPPose())
    print("tcp pose ", rtde_r.getActualTCPPose())
    # move to the position in front of the buttonn
    print(button_location_l)

    inp = input("Move to the position in front of the button? y/n: ")[0]
    in_front = deepcopy(button_location_l)

    if (inp == 'y'):
        
        rtde_c.moveL(in_front, 0.1, 0.01)
        time.sleep(5)
    else:
        print ("Skipping")

    btn.update_button()
    button_location_l = btn.get_button()
    print(rtde_r.getActualTCPPose())
    print("tcp pose ", rtde_r.getActualTCPPose())
    # move to the position in front of the buttonn
    print(button_location_l)

    inp = input("Move to the position in front of the button? y/n: ")[0]
    in_front = deepcopy(button_location_l)

    if (inp == 'y'):
        
        rtde_c.moveL(in_front, 0.1, 0.01)
        time.sleep(5)
    else:
        print ("Skipping")
    
    #parameters for logging
    t_list = []
    q_list = []
    qd_list = []
    t_list.append(time.time())
    q_list.append(rtde_r.getActualQ())
    qd_list.append(rtde_r.getActualQd()) 

    #parameters for servoing
    lf_list = []
    lf_list.append( rtde_r.getActualTCPForce() )
    dt = 0.024
    lookahead_time = 0.03
    gain = 100
    new_pos = rtde_r.getActualTCPPose()
    new_pos[1] -= 0.001

    # check servo push
    inp = input("Check forced push? y/n: ")[0]
    if (inp == 'y'):
        t_list.append(time.time())
        q_list.append(rtde_r.getActualQ())
        qd_list.append(rtde_r.getActualQd()) 
        lf_list.append( rtde_r.getActualTCPForce() )
        for i in range(200):  
            start = time.time()
            rtde_c.servoL(new_pos, 0.5, 0.25, dt, lookahead_time, gain)
            lf_list.append( rtde_r.getActualTCPForce() )
            t_list.append(time.time())
            q_list.append(rtde_r.getActualQ())
            qd_list.append(rtde_r.getActualQd()) 
            if lf_list[i][1] > 60:
                break
            new_pos = rtde_r.getActualTCPPose()
            new_pos[1] -= 0.001
            end = time.time()
            duration = end - start
            if duration < dt:
                time.sleep(dt - duration)
        rtde_c.servoStop()
    else:
        print ("Skipping")



    # release the button
    if move_to_operate_point(rtde_c, rtde_r):
        t_list.append(time.time())
        q_list.append(rtde_r.getActualQ())
        qd_list.append(rtde_r.getActualQd()) 
        lf_list.append( rtde_r.getActualTCPForce() )



    # fold the robot
    fold(rtde_c, rtde_r)

if __name__ == "__main__":
    main()