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
print("wait for service")
rospy.wait_for_service('get_buttons_pose')
try:
   get_buttons_pose = rospy.ServiceProxy('get_buttons_pose', GetButtons)
   pose = get_buttons_pose(1)
except rospy.ServiceException as e:
   print("Service call failed:", e)
   
UR_IP = "192.168.131.40"
rospy.init_node("press_button", anonymous=True, disable_signals=True)

rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
    rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)

# print actual joint position
actual_q = rtde_r.getActualQ()
print(actual_q)

# print actual cartesian position
actual_cart = rtde_r.getActualTCPPose()
print(actual_cart)
in_front = deepcopy(actual_cart)

# hardcoded button location
# button_location_l = [0.0777, -0.7158, 0.7498, -0.0188, 2.1946, -2.1974] #20/06/2021
# button_location_l = [0.0979, -0.63133, 0.7502, -0.0188, 2.1946, -2.1974] #21/06/2021 from SOLO
#button_location_l = [0.0389, -0.7473, 0.4283, -0.0188, 2.1946, -2.1974] #22/06/2021
#0.0389;-0.7473;0.4283
# Get the button center coordinate

#[0.03761258595316629, -0.6877641811981983, 0.5044145959004239, -0.03562794311799976, 2.2065298058858707, -2.1675840578837406]



target = np.array([pose.pose.position.x, pose.pose.position.y, pose.pose.position.z])
target_normal = np.array([pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z])
l = 0.2
h = 0.07
reals_to_tool = np.array([[1, 0, 0, -0.03], 
                          [0, 1, 0, -h],
                          [0, 0, 1, -l],
                          [0, 0, 0, 1]])
extra_len_p = np.concatenate([target, [1]])
point_rel_tool = np.matmul(reals_to_tool, extra_len_p)[0:3]
rot_mat = Rotation.from_rotvec(actual_cart[3:]).as_matrix()
trans_to_base = np.array([[1, 0, 0, actual_cart[0]], 
                          [0, 1, 0, actual_cart[1]],
                          [0, 0, 1, actual_cart[2]],
                          [0, 0, 0, 1]])
loc_button = np.matmul(trans_to_base, np.concatenate([np.matmul(rot_mat, point_rel_tool), [1]]))
normal_rel_base = target_normal

strange_rotation = Rotation.from_euler("xyz", [math.atan2(normal_rel_base[2],normal_rel_base[0]), -math.atan2(normal_rel_base[1],normal_rel_base[2]), 0]).as_rotvec()
#strange_rotation = np.matmul(rot_mat, strange_rotation)
#strange_rotation = Rotation.from_matrix(strange_rotation).as_rotvec()
print("target ",loc_button)
print("target normal", normal_rel_base)
#button_location_l = [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, -0.0188, 2.1946, -2.1974]
button_location_l = [loc_button[0], loc_button[1], loc_button[2], strange_rotation[0], strange_rotation[1], strange_rotation[2]]
print("tcp pose ", rtde_r.getActualTCPPose())
# move to intermediate
#intermediate_point_j = [1.3479, -1.9969, 1.5243, -2.6714, -1.3117, 0.0253]
intermediate_point_j = [1.5134, -2.2167, 1.6322, -2.5581, -1.4764, 0.0247]
inp = input("Move to intermediate point? y/n: ")[0]
if (inp == 'y'):    
    rtde_c.moveJ(intermediate_point_j, 0.1, 0.1)
else:
    print ("Skipping")

# move to the position in front of the buttonn
print(*button_location_l)
inp = input("Move to the position in front of the button? y/n: ")[0]
in_front = deepcopy(button_location_l)
#in_front[1] += 0.08
#in_front[2] += 0.06
if (inp == 'y'):
    rtde_c.moveL(in_front, 0.1, 0.01)
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
inp = input("Release the button? y/n: ")[0]
if (inp == 'y'):
    rtde_c.moveL(in_front, 0.1, 0.01)
    t_list.append(time.time())
    q_list.append(rtde_r.getActualQ())
    qd_list.append(rtde_r.getActualQd()) 
    lf_list.append( rtde_r.getActualTCPForce() )
    rtde_c.moveJ(intermediate_point_j, 0.1, 0.01)
else:
    print ("Skipping")

# save logs into the file
inp = input("Save parameter list to .csv? y/n: ")[0]
if (inp == 'y'):
    date_time = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    dotcsv = '.csv'
    filename = date_time + dotcsv
    with open(filename, 'x', newline='') as csvfile:
        param_writer = csv.writer(csvfile, delimiter=' ', quotechar='|', quoting=csv.QUOTE_MINIMAL)
        param_writer.writerow( ['button_location',  button_location_l[0], button_location_l[1], 
                        button_location_l[2], button_location_l[3], button_location_l[4], button_location_l[5] ] )
        param_writer.writerow( ['number', 'timestamp',
                        'position_0', 'position_1', 'position_2', 'position_3', 'position_4', 'position_5', 
                        'velocity_0', 'velocity_1', 'velocity_2', 'velocity_3', 'velocity_4', 'velocity_5',
                        'force_0', 'force_1', 'force_2', 'force_3', 'force_4', 'force_5', 
                        ] )
        list_size = len(t_list)
        for i in range (list_size):            
            param_writer.writerow ([ i, t_list[i], 
                        q_list[i][0], q_list[i][1], q_list[i][2], q_list[i][3], q_list[i][4], q_list[i][5], 
                        qd_list[i][0], qd_list[i][1], qd_list[i][2], qd_list[i][3], qd_list[i][4], qd_list[i][5], 
                        lf_list[i][0], lf_list[i][1], lf_list[i][2], lf_list[i][3], lf_list[i][4], lf_list[i][5], 
                        ])            
else:
    print ("Skipping")

#np = input("Print the force list? y/n: ")[0]
#if (inp == 'y'):
#    i = 0
#    for lf in lf_list:
#        print(i, lf)
#        i += 1
#else:
#    print ("Skipping")


# fold the robot
inp = input("Fold the robot? y/n: ")[0]
if (inp == 'y'):
    folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
    rtde_c.moveJ(folded_joints, 0.1, 0.1)
else:
    print ("Skipping")
