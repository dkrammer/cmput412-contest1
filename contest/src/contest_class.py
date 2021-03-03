#! /usr/bin/env python

import rospy
import time
from movement_class import MovementClass
from camera_service_server import Camera_utilities

class ContestClass():
    def __init__(self):
        self.move_obj = MovementClass()
        self.cam_obj = Camera_utilities()
    
    def move(self, path):
        for point in path:
            x, y = point
            self.move_obj.go_to(x, y)

    def task_one(self):
        
        path = [(-7, -8), (-7, -5.5)]
        self.move(path)
        rospy.loginfo("Stopped at stop sign 1")
        rospy.loginfo("Entering task 1")
        
        self.move([(-3, -5.5)]) # Values work for suspect
        rospy.loginfo("Scanning")
        
        self.move_obj.face_point(0, -5.2)

        if self.cam_obj.detect_wanted():
            rospy.loginfo("Suspect identified")
        else:
            rospy.loginfo("Not the suspect")
        
        rospy.loginfo("Scan complete")
        rospy.loginfo("Exiting task 1")

        self.move([(-8, -5.5)])
        rospy.loginfo("Exited task 1")
    
    def task_two(self):
        path = [(-7, 7.5), (-5, 7.5)]
        self.move(path)
        rospy.loginfo("Stopped at stop sign 2")
        time.sleep(0.3)
        rospy.loginfo("Skipping task 2")

    def task_three(self):
        path = [(7.5, 7), (7.5, -7.5), (5.5, -8)]
        self.move(path)
        rospy.loginfo("Stopped at stop sign 3")
        rospy.loginfo("Entering task 3")
        
        # Takes picture of cube
        self.move([(5.5, -5.8)])
        rospy.loginfo("Started task 3")
        self.move([(4.1, -5.8)])
        self.move_obj.face_point(0, -5.9)
        time.sleep(0.5)
        self.cam_obj.take_picture_of_cube()
        rospy.loginfo("Took picture of key cube")

        number = self.cam_obj.get_number_of_block()
        rospy.loginfo("Number detected: " + str(number))

        # Moves in front of cubes
        self.move([(4.1, 1)])
        rospy.loginfo("Moved in front of cubes")

        rospy.loginfo("Scanning Cubes")
        location = self.cam_obj.which_cube_is_it('/home/user/catkin_ws/src/cmput412-contest1/contest/src/key_cube.jpg')
        rospy.loginfo("It is the " + location + " cube")

        if location == "left":
            self.move([(2, 1), (2, 3)])
            self.move_obj.face_point(2, 5)
            rospy.loginfo("Scanning Left Cube")
            left_num = self.cam_obj.get_number_of_block()
            rospy.loginfo("Number " + str(left_num) + " found on left block")
            if left_num == number:
                rospy.loginfo("Number " + str(left_num) + " confirmed on left block")
                rospy.loginfo("Exiting task 3")
                return
            else:
                rospy.loginfo("Number " + str(number) + " not found")
                rospy.loginfo("Number " + str(number) + " not present in room")
                rospy.loginfo("Exiting task 3")
           
        elif location == "center":
            self.move([(3.9, 3)])
            self.move_obj.face_point(3.9, 5)
            rospy.loginfo("Scanning Center Cube")
            center_num = self.cam_obj.get_number_of_block()
            rospy.loginfo("Number " + str(center_num) + " found on center block")
            
            if center_num == number:
                rospy.loginfo("Number " + str(center_num) + " confirmed on center block")
                rospy.loginfo("Exiting task 3")
                return
            else:
                rospy.loginfo("Number " + str(number) + " not found")
                rospy.loginfo("Number " + str(number) + " not present in room")
                rospy.loginfo("Exiting task 3")

        elif location == "right":
            self.move([(5.5, 1), (5.5, 3)])
            self.move_obj.face_point(5.5, 5)
            rospy.loginfo("Scanning Right Cube")
            right_num = self.cam_obj.get_number_of_block()
            rospy.loginfo("Number " + str(right_num) + " found on right block")
            
            if right_num == number:
                rospy.loginfo("Number " + str(right_num) + " confirmed on right block")
                rospy.loginfo("Exiting task 3")
                return
            else:
                rospy.loginfo("Number " + str(number) + " not found")
                rospy.loginfo("Number " + str(number) + " not present in room")
                rospy.loginfo("Exiting task 3")
        
        rospy.loginfo("Exiting task 3")
    
    def task_four(self):
        self.move([(6, -8)])
        rospy.loginfo("Exited task 3")
        self.move([(-0.5, -8.3)])
        rospy.loginfo("Stopped at stop sign 4")
        rospy.loginfo("End of loop.")

        
