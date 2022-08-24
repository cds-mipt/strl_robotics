#!/usr/bin/env python3
import rospy
import time
from manipulator.srv import PressButton, PressButtonResponse 
rospy.init_node("service_press_tester")
rospy.wait_for_service("press_button")
press = rospy.ServiceProxy("press_button", PressButton)

print(press(zmin = 0, zmax = 1.5, base_joint = -0.5, end_operate = True))
