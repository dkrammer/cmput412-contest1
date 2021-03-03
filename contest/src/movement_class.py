#! /usr/bin/env python

import rospy
import math
import time
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from datetime import timedelta


class MovementClass():
    def __init__(self):
        
        self.odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.move = Twist()
        
        self.linear_velocity = 0.8
        self.face_point_vel = 0.8

        self.current_pose = {}
        self.exit_topic = False
        self.freq = 10 # 10hz
        self.rate = rospy.Rate(self.freq)

    
    def odom_callback(self, msg):
        self.current_pose['x'] = msg.pose.pose.position.x
        self.current_pose['y'] = msg.pose.pose.position.y
        self.current_pose['theta'] = euler_from_quaternion([msg.pose.pose.orientation.x,
                                                           msg.pose.pose.orientation.y,
                                                           msg.pose.pose.orientation.z,
                                                           msg.pose.pose.orientation.w])[2]

    def publish_info(self):
        # Method taken from "Using Python Classes in ROS" segment
        while not self.exit_topic:
            connections = self.vel_pub.get_num_connections()
            if connections > 0:
                self.vel_pub.publish(self.move)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdown(self):
        self.stop() # Make turtle stop on shutdown
        self.exit_topic = True

    def stop(self):
        self.move.linear.x = 0
        self.move.angular.x = 0
        self.publish_info()
        #rospy.loginfo("Stopped")

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.vel_pub.publish(msg)


    # returns the distance between two points
    def distance(self, point1, point2):
        return math.sqrt((point2[0] - point1[0])**2 + (point2[1] - point1[1])**2)


    # Normalize function taken from section 5, Kinematic Control
    def normalize(self, angle):
        return math.atan2(math.sin(angle),math.cos(angle))


    def go_to(self, x_goal, y_goal):
        
        constant_vel = self.linear_velocity
        target_theta = self.face_point(x_goal, y_goal)

        k_rho = 0.1
        k_alpha = 1.0
        k_beta = -0.15
        rho = float("inf")
        while rho > 0.1:
            dx = x_goal - self.current_pose['x']
            dy = y_goal - self.current_pose['y']
            theta = self.current_pose['theta']

            rho = np.sqrt(dx**2 + dy**2)
            alpha = self.normalize(math.atan2(dy, dx) - theta)
            beta = self.normalize(target_theta - math.atan2(dy, dx))
            v = k_rho * rho
            w = k_alpha * alpha + k_beta * beta

            # scale linear and angular velocity to keep robot
            # moving at a constant linear velocity
            abs_v = abs(v)
            v = v / abs_v * constant_vel
            w = w / abs_v * constant_vel
            self.move.linear.x = v
            self.move.angular.z = w
            self.publish_info()
            self.rate.sleep()
        self.stop()
        time.sleep(0.2)
    
    #this will ensure that no matter where the robot ends up after the spiral
    #it will point directly at the next point
    #preventing crashes
    def face_point(self, x, y):
        difference = 100.0
        theta, target_theta = self.get_thetas(x, y)
        rotation_factor = self.get_rotation_factor(theta, target_theta)
        while difference > 0.25:
            theta, target_theta = self.get_thetas(x, y)
            difference = abs(theta - target_theta)
            self.move.angular.z = self.face_point_vel * rotation_factor
            self.publish_info()
            rospy.sleep(0.01)
        self.move.angular.z = 0
        self.publish_info()
        time.sleep(0.1)
        return target_theta

    def get_thetas(self, x, y):
        theta = self.current_pose['theta']
        dx = x - self.current_pose['x']
        dy = y - self.current_pose['y']
        target_theta = np.arctan2(dy, dx)
        return theta, target_theta

    def get_rotation_factor(self, theta, target_theta):
            theta = self.convert_theta(theta)
            target_theta = self.convert_theta(target_theta)
            
            alpha = target_theta - theta
            beta = target_theta - theta + 2 * np.pi
            gamma = target_theta - theta - 2 * np.pi
            
            original = [alpha, beta, gamma]
            absolute = [abs(alpha), abs(beta), abs(gamma)]
            
            idx = np.argmin(absolute)           
            if original[idx] >= 0: return 1
            else: return -1
    
    def convert_theta(self, theta):
        if theta < 0:
            return theta + 2 * np.pi
        else:
            return theta


if __name__ == '__main__':
    rospy.init_node('movement_node', anonymous=True)
    move_obj = MovementClass()
    time.sleep(0.05)
    move_obj.go_to(-0.6, -8.1, -180)
    move_obj.stop()
    try: 
        path = [(-7, -8, -180), (-7, -5.5, 90), (-2, -5.5, 0), (-8, -5.5, -180),
                (-7, 7.5, 90), (-5, 7.5, 0), (7.5, 7, 0), (7.5, -7.5, -90), 
                (5.5, -7, 90), (5.5, -6, -180), (3.5, -6, -180), (4, 3, 90), 
                (6, -8, -90), (0, -8, -180)]
        for point in path:
            x, y, th = point
            move_obj.go_to(x, y, th)
        
    except rospy.ROSInterruptException:
        move_obj.stop()
