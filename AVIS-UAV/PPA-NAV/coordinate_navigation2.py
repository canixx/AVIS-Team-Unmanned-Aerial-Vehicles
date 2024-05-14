# KOD MAVROS İLE BİRLEŞTİRİLECEK HAZIR DURUMDA DEĞİL.
# ALİ LİVAN TÜRK


#!/usr/bin/env python
import time
import rospy
from rospy import Rate
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, Point, Twist
from std_msgs.msg import String
import math
import heapq

# BU KISIMDA INIT_NODE HATASI VAR SÜRÜM FARKI OLABİLİR ARAŞTIR.
rospy.init_node('navigation_node')

# Map datasını ve goal positionu tanımlayan yer.
map_data = None
goal_pose = PoseStamped()

# Cell valuelerin olduğu kısım. Duruma göre burdaki mantık değiştirilebilir.
# MAVROS kısmı bitirildiğinde burayı düzenle 
CELL_UNKNOWN = -1
CELL_FREE = 0
CELL_OCCUPIED = 100

# Drone hareketinin değerleri
# Bu kısım mavros ile alakalı tekrar düzenle ya da sil.
FORWARD_SPEED = 0.2
ROTATE_SPEED = 0.5

# Map datamızı işleyen fonksiyon
def map_callback(data):
    global map_data
    map_data = data

# Map datasını almak için subscribe olma
rospy.Subscriber('/map', OccupancyGrid, map_callback)

# Goal positionu işleyen fonksiyon
def goal_callback(data):
    global goal_pose
    goal_pose = data

# Goal topicine subscribe eden kod.
rospy.Subscriber('/goal', PoseStamped, goal_callback)

# Koordinat noktalarını cell indexlere dönüştüren kod
def coordinates_to_indices(x, y):
    global map_data
    resolution = map_data.info.resolution
    offset_x = map_data.info.origin.position.x
    offset_y = map_data.info.origin.position.y
    i = int(round((x - offset_x) / resolution))
    j = int(round((y - offset_y) / resolution))
    return i, j

# Cell indexlerini koordinat noktalarına dönüştüren kod.
def indices_to_coordinates(i, j):
    global map_data
    resolution = map_data.info.resolution
    offset_x = map_data.info.origin.position.x
    offset_y = map_data.info.origin.position.y
    x = (i * resolution) + offset_x + (resolution / 2)
    y = (j * resolution) + offset_y + (resolution / 2)
    return x, y

# Map datasında aldığımız cellerin cell valuelerini basan kod.
def get_cell_value(i, j):
    global map_data
    width = map_data.info.width
    index = (j * width) + i
    cell_value = map_data.data[index]
    return cell_value

# 2 cell arasındaki mesafeyi bulan kod, euclidian algorithm kullanıyor.
def get_distance(x1, y1, x2, y2):
    dx = x1 - x2
    dy = y1 - y2
    distance = math.sqrt(dx ** 2 + dy ** 2)
    return distance

# En uzaktaki boş celli bulan algoritma, bütün temel burda.
def get_farthest_free_cell(x, y):
    global map_data
    max_distance = get_distance(x, y, indices_to_coordinates(map_data.info.width - 1, map_data.info.height - 1)[0], indices_to_coordinates(map_data.info.width - 1, map_data.info.height - 1)[1])
    farthest_node = None
    farthest_dist = -1
    queue = []
    visited = set()
    start_indices = coordinates_to_indices(x, y)
    start_node = (start_indices, 0)
    heapq.heappush(queue, start_node)
    while queue:
        curr_node = heapq.heappop(queue)
        curr_indices = curr_node[0]
        curr_dist = curr_node[1]
        if curr_indices in visited:
            continue
        visited.add(curr_indices)
        curr_x, curr_y = indices_to_coordinates(curr_indices[0], curr_indices[1])
        curr_value = get_cell_value(curr_indices[0], curr_indices[1])
        if curr_value == CELL_FREE:
            distance = get_distance(curr_x, curr_y, x, y)
            if distance > farthest_dist:
                farthest_node = curr_indices
                farthest_dist = distance
            if farthest_dist == max_distance:
                break
        elif curr_value == CELL_UNKNOWN or curr_value == CELL_OCCUPIED:
            continue
        for i in range(-1, 2):
            for j in range(-1, 2):
                if i == 0 and j == 0:
                    continue
                next_indices = (curr_indices[0] + i, curr_indices[1] + j)
                if next_indices[0] < 0 or next_indices[1] < 0 or next_indices[0] >= map_data.info.width or next_indices[1] >= map_data.info.height:
                    continue
                next_value = get_cell_value(next_indices[0], next_indices[1])
                if next_value == CELL_UNKNOWN or next_indices in visited:
                    continue
                next_dist = curr_dist + 1
                next_node = (next_indices, next_dist)
                heapq.heappush(queue, next_node)
    if farthest_node is not None:
        farthest_x, farthest_y = indices_to_coordinates(farthest_node[0], farthest_node[1])
        return farthest_x, farthest_y
    else:
        return None

def stop_robot():
    twist = Twist()
    cmd_vel_pub.publish(twist)

# Bu kısım MAVROS ile alakalı, değiştirilecek ya da silinecek.
def navigate(dx, dy):
    global FORWARD_SPEED, ROTATE_SPEED, goal_pose, cmd_vel_pub

    rospy.loginfo("Navigating to cell ({}, {})".format(dx, dy))
    distance = get_distance(0, 0, dx, dy)
    angle = math.atan2(dy, dx)
    quaternion = quaternion_from_euler(0, 0, angle)
    goal_pose.pose.position = Point(0, 0, 0)
    goal_pose.pose.orientation.x = quaternion[0]
    goal_pose.pose.orientation.y = quaternion[1]
    goal_pose.pose.orientation.z = quaternion[2]
    goal_pose.pose.orientation.w = quaternion[3]
    cmd_vel = Twist()

    while distance > 0.1:
        cmd_vel.linear.x = min(distance, FORWARD_SPEED)
        cmd_vel.angular.z = ROTATE_SPEED * angle
        cmd_vel_pub.publish(cmd_vel)

        rate = Rate(10)  # 10 Hz
        rate.sleep()

        x = goal_pose.pose.position.x
        y = goal_pose.pose.position.y
        distance = get_distance(x, y, dx, dy)
        angle = math.atan2(dy - y, dx - x)

    stop_robot()