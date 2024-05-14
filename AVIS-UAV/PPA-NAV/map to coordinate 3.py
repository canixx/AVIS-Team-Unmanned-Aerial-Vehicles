#!/usr/bin/env python

import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_callback(msg):
    map_data = np.array(msg.data).reshape(msg.info.height, msg.info.width)
    # Convert to bottom-left origin
    map_data = np.flipud(map_data)
    # Convert to x-y coordinate system
    x_coord = np.arange(0, msg.info.resolution * msg.info.width, msg.info.resolution)
    y_coord = np.arange(0, msg.info.resolution * msg.info.height, msg.info.resolution)
    x_mesh, y_mesh = np.meshgrid(x_coord, y_coord)
    x_coords = x_mesh.flatten()
    y_coords = y_mesh.flatten()
    cell_values = map_data.flatten()
    # Save to file
    np.savetxt('/home/livan/Desktop/map_coordinates.txt', np.column_stack((x_coords, y_coords, cell_values)), header='x, y, cell_value', comments='', fmt='%.2f')

if __name__ == '__main__':
    rospy.init_node('map_subscriber')
    rospy.Subscriber('/map', OccupancyGrid, map_callback)
    rospy.spin()
