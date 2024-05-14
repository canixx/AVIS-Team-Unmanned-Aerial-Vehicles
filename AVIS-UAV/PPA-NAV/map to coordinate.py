import rospy
from nav_msgs.msg import OccupancyGrid
import numpy as np

def map_callback(msg):
    # Extract the map data from the message
    map_data = msg.data

    # Convert the data to a 2D numpy array
    map_matrix = np.array(map_data, dtype=np.int8).reshape((msg.info.height, msg.info.width))

    # Calculate the X and Y coordinates of each point in the occupancy grid
    x_coords, y_coords = np.meshgrid(np.arange(-msg.info.resolution*msg.info.width/2, msg.info.resolution*msg.info.width/2, msg.info.resolution),
                                     np.arange(-msg.info.resolution*msg.info.height/2, msg.info.resolution*msg.info.height/2, msg.info.resolution))

    # Shift the coordinates so that the origin is at the center of the occupancy grid
    x_coords = x_coords - (msg.info.resolution*msg.info.width/2)
    y_coords = y_coords - (msg.info.resolution*msg.info.height/2)

    # Save the coordinates to a file
    np.savetxt("/home/livan/Desktop/map_matrix.txt", np.column_stack((x_coords.flatten(), y_coords.flatten())), delimiter=" ", fmt='%e')


# Initialize the node
rospy.init_node('map_listener')

# Subscribe to the map topic
sub = rospy.Subscriber('map', OccupancyGrid, map_callback)

# Spin to keep the node running
rospy.spin()
