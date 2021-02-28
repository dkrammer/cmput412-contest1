#! /usr/bin/env python

import rospy 
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
import matplotlib.pyplot as plt

#TODO import a suitable message. 
#from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.

#TODO put in class/ functions that can do the things we need.
class Camera_utilities(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()
        self.cv_image = None
        self.cube_image = None
        

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            self.cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)
        self.feature_detection()

    def take_picture_of_cube(self):
        self.cube_image = np.copy(self.cv_image)

    #TODO implement this
    def filter_colors(self, img, color):
        return False

    #TODO add picture parameters      
    #TODO determine if the camera is the same as picture
    def feature_detection(self):    

        image_1 = cv2.imread('/home/user/catkin_ws/src/cmput412-contest1/contest/src/wanted_cropped.jpg',1)
        image_2 = self.cv_image
       
        
        #Size for the image 
        imX = 700
        imY = 500

        #img_2 = cv2.resize(cv_image,(imX,imY))
        img_2 = cv2.resize(self.cv_image,(imX,imY))
        image_2 = img_2

        

        

        

        gray_1 = cv2.cvtColor(image_1, cv2.COLOR_RGB2GRAY)
        gray_2 = cv2.cvtColor(image_2, cv2.COLOR_RGB2GRAY)

        #Initialize the ORB Feature detector 
        orb = cv2.ORB_create(nfeatures = 2000)

        #Make a copy of the original image to display the keypoints found by ORB
        #This is just a representative
        preview_1 = np.copy(image_1)
        preview_2 = np.copy(image_2)

        #Create another copy to display points only
        dots = np.copy(image_1)

        #Extract the keypoints from both images
        train_keypoints, train_descriptor = orb.detectAndCompute(gray_1, None)
        test_keypoints, test_descriptor = orb.detectAndCompute(gray_2, None)

        #Draw the found Keypoints of the main image
        cv2.drawKeypoints(image_1, train_keypoints, preview_1, flags = cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
        cv2.drawKeypoints(image_1, train_keypoints, dots, flags=2)

        #############################################
        ################## MATCHER ##################
        #############################################

        #Initialize the BruteForce Matcher
        bf = cv2.BFMatcher(cv2.NORM_HAMMING2, crossCheck = True)
        #bf = cv2.FlannBasedMatcher.create()
        #bf = cv2.DescriptorMatcher_create(cv2.DescriptorMatcher_FLANNBASED)
        #bf = cv2.DescriptorMatcher_FLANNBASED
        #Match the feature points from both images
        matches = bf.match(train_descriptor, test_descriptor)

        #The matches with shorter distance are the ones we want.
        matches = sorted(matches, key = lambda x : x.distance)
        #Catch some of the matching points to draw
        
            
        good_matches = matches[:390] 
        

        #Parse the feature points
        train_points = np.float32([train_keypoints[m.queryIdx].pt for m in good_matches]).reshape(-1,1,2)
        test_points = np.float32([test_keypoints[m.trainIdx].pt for m in good_matches]).reshape(-1,1,2)

        #Create a mask to catch the matching points 
        #With the homography we are trying to find perspectives between two planes
        #Using the Non-deterministic RANSAC method
        M, mask = cv2.findHomography(train_points, test_points, cv2.RANSAC, 5.0)
        # cv2.RANSAC,5.0
        #print("start of m")
        #print(len(mask))
        #print("end of m")
        #length of mask when hit is 137 
        #when not detected its around 70

        if (len(mask) > 120):
            print("detected wanted person")
        else:
            print("not detected")

        #Catch the width and height from the main image
        h,w = gray_1.shape[:2]

        #Create a floating matrix for the new perspective
        pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        #Create the perspective in the result 
        dst = cv2.perspectiveTransform(pts,M)

        #Draw the matching lines 
        

        # Draw the points of the new perspective in the result image (This is considered the bounding box)
        result = cv2.polylines(image_2, [np.int32(dst)], True, (50,0,255),3, cv2.LINE_AA)

        #addition = cv2.add(img_2,image_2)
        cv2.imshow('image',img_2)
        cv2.imshow('Points',preview_1)
        
        cv2.imshow('Detection',image_2)       
        #cv2.imshow('Detection',addition)    

        cv2.waitKey(1)


def server_callback(request):
    print("my callback has been called")
    #depending on what is in the message, do a certian thing

    return EmptyResponse()

rospy.init_node('camera_service_server')
#my_service = rospy.Service('/camera_functions', Empty, server_callback)
c_util = Camera_utilities()
rospy.spin()