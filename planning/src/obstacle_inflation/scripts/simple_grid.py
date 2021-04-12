#!/usr/bin/env python
# license removed for brevity
import rospy
import numpy as np
import random
from nav_msgs.msg import OccupancyGrid

grid = OccupancyGrid()
grid.header.frame_id = "map"
grid.info.resolution = 0.2
grid.info.origin.orientation.w = 1
grid.info.height = 523
grid.info.width = 1030

orig_x = 143
orig_y = 140
size_x = 177
size_y = 120

grid.data = np.zeros(grid.info.height*grid.info.width, np.int8)
for x in range(orig_x, orig_x+size_x):
    for y in range(orig_y, orig_y+size_y):
        grid.data[y*grid.info.width + x] = 100 if random.random() > 0.95 else 0

def talker():
    pub = rospy.Publisher('simple_grid', OccupancyGrid, queue_size=10)
    rospy.init_node('simple_grid_publisher', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():

        pub.publish(grid)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass