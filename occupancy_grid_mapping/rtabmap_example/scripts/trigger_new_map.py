#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty


if __name__ == '__main__':
    rospy.init_node('trigger_new_map')
    rospy.wait_for_service('trigger_new_map')
    try:
        trigger_new_map = rospy.ServiceProxy('trigger_new_map', Empty)
        trigger_new_map()
    except rospy.ServiceException:
        print('Something\'s gone wrong.')

