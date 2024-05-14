import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_callback(msg):
    # Extract the map data from the message
    map_data = msg.data

    # Convert the data to a 2D numpy array
    map_matrix = np.array(map_data, dtype=np.int8).reshape((msg.info.height, msg.info.width))
    np.savetxt("/home/livan/Desktop/map_matrix.txt", map_matrix, delimiter=" ", fmt='%e')


# Initialize the node
rospy.init_node('map_listener')

# Subscribe to the map topic
sub = rospy.Subscriber('map', OccupancyGrid, map_callback)

# Spin to keep the node running
rospy.spin()