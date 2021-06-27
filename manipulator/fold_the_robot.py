import rtde_control
import rtde_receive
import time
from std_msgs.msg import String
from copy import deepcopy

UR_IP = "192.168.131.40"

rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
    rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)

inp = input("Fold the robot? y/n: ")[0]
if (inp == 'y'):
    folded_joints = [1.602, -2.869, 2.683, -2.869, -1.584, -0.001]
    rtde_c.moveJ(folded_joints, 0.1, 0.01)
else:
    print ("Skipping")