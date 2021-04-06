#!/usr/bin/env python

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np


def scan_callback(msg):
    global center
    global left
    global right
    for i in range(15):
        center.append(msg.ranges[345+i])
    for i in range(15)
        center.append(msg.ranges[0+i])
    right = msg.ranges[30]
    left = msg.ranges[330]



def odom_callback(msg):
    global pose
    pose = msg.pose

#set up nodes
rospy.init_node('avoider')
watching = rospy.Subscriber('/scan', LaserScan, scan_cb)
pub = rospy.Publisher('cmd_vel', Twist, queue_size = 1)

#set up variables
pose = None
rate = rospy.Rate(10)

# buffer
while pose == None
    continue

#Main Program
t = Twist()

#function for reporting obstacles
def are_we_clear()
    for i in range(30)
        if center.ranges[i] < .5:
            return False
    return True

#funtion deciding which direction robot should turn
def which_direction()
    while (center)
        if (right > left):
            return 1 # will be used turn to decide to turn righ
    return 0 # willl be used to turn left. favors left to move faster

#actually spin the robot
def turning (a)
    while not are_we_clear(): #checks to see if the way is clear for the robot
        if a == 1: # recall 1 = right 
            t.angular.x = -.5
            moving.publish(t)
        else:
            t.angular.x = -.5
            moving.publish(t)
