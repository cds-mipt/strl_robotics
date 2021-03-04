#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid

orig_x = 50
orig_y = 50
size = 25
def talker():
    pub = rospy.Publisher('simple_grid', OccupancyGrid, queue_size=10)
    rospy.init_node('simple_grid_publisher', anonymous=True)
    rate = rospy.Rate(5) # 10hz
    while not rospy.is_shutdown():
        grid = OccupancyGrid()
        grid.header.frame_id = "map"
        grid.info.resolution = 0.2
        grid.info.origin.orientation.w = 1
        grid.info.height = 150
        grid.info.width = 100
        grid.data = np.zeros(150*100, np.int8)
        for x in range(orig_x, orig_x+size):
            for y in range(orig_y, orig_y+size):
                grid.data[y*100 + x] = 100
        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass