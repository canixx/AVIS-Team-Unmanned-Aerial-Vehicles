import rospy
from sensor_msgs.msg import LaserScan
import numpy as np

def scan_callback(msg):
    ranges = np.array(msg.ranges)
    angles = np.linspace(msg.angle_min, msg.angle_max, len(ranges))
    scan_matrix = np.vstack((ranges * np.cos(angles), ranges * np.sin(angles)))
    np.savetxt("/home/livan/Desktop/scan_matrix.txt", scan_matrix, delimiter=" ", fmt='%e')

rospy.init_node('laser_scan_matrix')
scan_subscriber = rospy.Subscriber('/scan', LaserScan, scan_callback)
rospy.spin()
