import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_callback(msg):
    map_data = msg.data
    map_matrix = np.array(map_data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    np.savetxt("/home/livan/Desktop/map_matrix.txt", map_matrix, delimiter=" ", fmt='%e')

rospy.init_node('map_listener')
sub = rospy.Subscriber('map', OccupancyGrid, map_callback)

rospy.spin()