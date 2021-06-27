import rtde_control
import rtde_receive
import time
from std_msgs.msg import String

UR_IP = "192.168.131.40"

def free_drive():
    global rtde_c

    # start freedrive mode
    inp = input("Start freedrive mode? y/n: ")[0]
    if (inp == 'y'):
        rtde_c.teachMode()
        print ("Freedrive mode is on")
    else:
        print ("Skipping")
        return

    # stop freedrive mode
    inp = input("Stop freedrive mode? y/n: ")[0]
    if (inp == 'y'):
        rtde_c.endTeachMode()
        print ("Freedrive mode is off")
    else:
        print ("Freedrive mode will remain active after execution of this program")
        return

if __name__ == '__main__':
    rtde_c = rtde_control.RTDEControlInterface(UR_IP, 
        rtde_control.RTDEControlInterface.FLAG_VERBOSE | rtde_control.RTDEControlInterface.FLAG_USE_EXT_UR_CAP)
    rtde_r = rtde_receive.RTDEReceiveInterface(UR_IP)
    free_drive()
    
