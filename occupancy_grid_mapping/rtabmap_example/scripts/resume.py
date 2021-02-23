#!/usr/bin/env python3
import rospy
from std_srvs.srv import Empty


if __name__ == '__main__':
    rospy.init_node('resume')
    rospy.wait_for_service('/rtabmap/resume')
    try:
        trigger_new_map = rospy.ServiceProxy('/rtabmap/resume', Empty)
        trigger_new_map()
    except rospy.ServiceException:
        print('Something\'s gone wrong.')

