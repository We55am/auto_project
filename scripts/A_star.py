#!/usr/bin/env python
import cv2
import cell as nodeObj
import numpy as np
from pprint import pprint
import Queue
import rospy
from geometry_msgs.msg import Twist,Pose
from nav_msgs.msg import Odometry
import math

def euler_to_quaternion(yaw, pitch, roll):
    x = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    y = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    z = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    w = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return [x, y, z, w]

def quaternion_to_euler(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return [yaw, pitch, roll]

def generateChildren(parent, open_list, open_list_helper, dis_mat):
    parent.list_check = True
    p_x = parent.LocationX
    p_y = parent.LocationY
##    #RIGHT NEIGHBOUR CELL
    if p_x+1 < w and not parent.obstacle:
        childern = grid[p_y][p_x+1]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y][p_x+1])
##    #NORTH-EAST NEIGHBOUR CELL
    if p_x+1 < w and p_y-1 >= 0 and not parent.obstacle:
        childern = grid[p_y-1][p_x+1]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y-1][p_x+1])
##    #UP NEIGHBOUR CELL
    if p_y-1 >= 0 and not parent.obstacle:
        childern = grid[p_y-1][p_x]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y-1][p_x])
##    #NORTH-WEST NEIGHBOUR CELL
    if p_y-1 >= 0 and p_x-1 >= 0 and not parent.obstacle:
        childern = grid[p_y-1][p_x-1]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y-1][p_x-1])
##    #LEFT NEIGHBOUR CELL
    if p_x-1 >= 0 and not parent.obstacle:
        childern = grid[p_y][p_x-1]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y][p_x-1])
##    #SOUTH-WEST NEIGHBOUR CELL
    if p_x-1 >= 0 and p_y+1 < h and not parent.obstacle:
        childern = grid[p_y+1][p_x-1]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y+1][p_x+1])
##    #DOWN NEIGHBOUR CELL
    if p_y+1 < h and not parent.obstacle:
        childern = grid[p_y+1][p_x]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y+1][p_x])
##    #SOUTH-EAST NEIGHBOUR CELL
    if p_y+1 < h and p_x+1 < w and not parent.obstacle:
        childern = grid[p_y+1][p_x+1]
        if not childern.obstacle and not childern.list_check:
            childern.list_check = True
            childern.parent = parent
            open_list.append(childern)
            open_list_helper.append(dis_mat[p_y+1][p_x+1])
            
def callback(data):
    global pos_msg
    global vel_msg
    pos_msg = data.pose.pose
    vel_msg = data.twist.twist

def gazebotogrid(y,x):
    return [(-y+2.0), (x+5.0)]
def gridtogazebo(y,x):
    return [(200-y), (x-500)]

def A_star():
    global START
    global GOAL
    result = []
    open_list = []
    open_list_helper = []
    closed_list = []
    dx_mat_goal = [0] * img
    dy_mat_goal = [0] * img
    dx_mat_start = [0] * img
    dy_mat_start = [0] * img
    dis_mat_goal = [0] * img
    dis_mat_start = [0] * img
    for i in range(h):
        for j in range(w):
            dy_mat_goal[i,j] = abs(y_Goal - i)
            dx_mat_goal[i,j] = abs(x_Goal - j)
            dy_mat_start[i,j] = abs(y_Start - i)
            dx_mat_start[i,j] = abs(x_Start - j)
            
    dis_mat_goal = np.sqrt(np.square(dx_mat_goal)+np.square(dx_mat_goal))
    dis_mat_start = np.sqrt(np.square(dx_mat_start)+np.square(dy_mat_start))
    dis_mat = dis_mat_goal + dis_mat_start
    current = START
    generateChildren(current, open_list, open_list_helper, dis_mat)
    
    while current is not GOAL:
        closed_list.append(current)
        mini_indx = np.argmin(open_list_helper)
        current = open_list[mini_indx]
        open_list.pop(mini_indx)
        open_list_helper.pop(mini_indx)
        generateChildren(current, open_list, open_list_helper, dis_mat)
    while current is not START:
        result.append(current)
        current = current.parent
    return result
    
if __name__=='__main__':
##    ROS Node Initialization
    rospy.init_node('Path_Planning', anonymous=True)
##    Publisher and topic Initialization
    pub1 = rospy.Publisher('/des_Pos', Pose, queue_size=10)
    pub_msg = Pose()
    rate = rospy.Rate(10)
##    Subscriber Initialization
    pos_msg = Pose()
    vel_msg = Twist()
    #sub1 = rospy.Subscriber('/odom', Odometry, callback)
    rospy.loginfo("The node is created !!")
##    Map Image Setups
    img_real = cv2.imread('./new_MAP.png', 0)
    img = cv2.imread('./temp.png', 0)
    h = len(img)
    w = len(img[0])

##    Algorithm input
    x_Start = 400
    y_Start = 100
    x_Goal = float(raw_input("Enter the goal value of X :"))
    y_Goal = float(raw_input("Enter the goal value of Y :"))
    [y_Goal, x_Goal] = gazebotogrid(y_Goal, x_Goal)
    y_Goal = int(y_Goal*100.0)
    x_Goal = int(x_Goal*100.0)
    if x_Start > w:
            print('Warning MSG: the start value of X should not exceed', w, '. Please enter a new start value.')
            x_Start = int(raw_input("Enter the start value of X :"))
    if y_Start > h:
            print('Warning MSG: the start value of Y should not exceed', h, '. Please enter a new start value.')
            y_Start = int(raw_input("Enter the start value of Y :"))
    if x_Goal > w:
            print('Warning MSG: the goal value of X should not exceed', w, '. Please enter a new start value.')
            x_Goal = int(raw_input("Enter the goal value of X :"))
    if y_Goal > h:
            print('Warning MSG: the goal value of Y should not exceed', h, '. Please enter a new start value.')
            y_Goal = int(raw_input("Enter the goal value of Y :"))

    if x_Start == x_Goal and y_Start == y_Goal:
            print('Warning MSG: You entered the same start and goal points. Please choose 2 different set of points')
            x_Start = int(raw_input("Enter the start value of X :"))
            y_Start = int(raw_input("Enter the start value of Y :"))
            x_Goal = int(raw_input("Enter the goal value of X :"))
            y_Goal = int(raw_input("Enter the goal value of Y :"))

    if img[y_Start,x_Start] == 0:
            print('Warning MSG: Sorry this start point is already occupied with an obstacle')
            x_Start = int(raw_input("Enter the start value of X :"))
            y_Start = int(raw_input("Enter the start value of Y :"))

    if img[y_Goal,x_Goal] == 0:
            print('Warning MSG: Sorry this goal point is already occupied with an obstacle')
            x_Goal = int(raw_input("Enter the goal value of X :"))
            y_Goal = int(raw_input("Enter the goal value of Y :"))

##    Making the Grid for path planning algorithms
    grid = nodeObj.make_grid(img)
    v = grid[y_Start][x_Start]
    GOAL = grid[y_Goal][x_Goal]
    START = grid[y_Start][x_Start]
    PATH = A_star()
    PATH.reverse()
 ##    for node_c in PATH:
##        img_real[node_c.LocationY][node_c.LocationX] = 100
##    cv2.imshow('TEST', img_real)
##    cv2.waitKey(10)
    rng = 0.005
    
    while True and not rospy.is_shutdown():
        for node in PATH:
            des_loc = gridtogazebo(node.LocationY, node.LocationX)
            des_x = float((des_loc[1])*0.010000)
            des_y = float((des_loc[0])*0.010000)

            while abs(pos_msg.position.x - des_x) > rng and abs(pos_msg.position.y - des_y) > rng:
                pub_msg.position.x = des_x
                pub_msg.position.y = des_y
                pub_msg.position.z = 0
                theta = float(math.atan(abs(pos_msg.position.y - des_y)/abs(pos_msg.position.x - des_x))*(180/math.pi))
                [pub_msg.orientation.x, pub_msg.orientation.y,
                 pub_msg.orientation.z,
                 pub_msg.orientation.w] = euler_to_quaternion(theta, 0, 0)
                pub1.publish(pub_msg)
            pub_msg.position.x = 0
            pub_msg.position.y = 0
            pub_msg.position.z = 0
            pub_msg.orientation.x = 0
            pub_msg.orientation.x = 0
            pub_msg.orientation.x = 0
            pub_msg.orientation.x = 0
            pub1.publish(pub_msg)
            rate.sleep()
        pass
    
