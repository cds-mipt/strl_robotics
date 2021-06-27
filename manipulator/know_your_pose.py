import rtde_control
import rtde_receive
import time
from scipy.spatial.transform import Rotation
from std_msgs.msg import String

UR_IP = "192.168.131.40"

rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
    rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)

# print actual joint position
actual_q = rtde_r.getActualQ()
print('joints:', actual_q)

# print actual cartesian position
actual_cart = rtde_r.getActualTCPPose()
print('cartesian:', actual_cart)

# print Euler angles
rot = Rotation.from_rotvec([ actual_cart[3], actual_cart[4], actual_cart[5] ])
print('Euler angles:', rot.as_euler('xyz') )