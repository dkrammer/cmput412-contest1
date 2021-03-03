#! /usr/bin/env python

'''
Script to get info about turtlebot's odometry
'''

import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

def callback(msg):
    
    #theta = euler_from_quaternion([msg.pose.pose.orientation.x,
    #                                                       msg.pose.pose.orientation.y,
    #                                                       msg.pose.pose.orientation.z,
    #                                                       msg.pose.pose.orientation.w])[2]
    
    #print msg.twist.twist.angular.z
    print msg.pose.pose.position

rospy.init_node('check_odometry')
odom_sub = rospy.Subscriber('/odom', Odometry, callback)
rospy.spin()