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
        #print(self.detect_wanted())
        #self.take_picture_of_cube()
        #test_cube_img = cv2.imread('/home/user/catkin_ws/src/cmput412-contest1/contest/src/test_cube.jpg',1)
        #self.feature_detection(test_cube_img,20,30,1000)
        print(self.which_cube_is_it())

    #Called when in middle of room and 4 meters away from cubes (-1,4,0)?
    #TODO Actually implement this
    def which_cube_is_it(self):
        test_cube_img = cv2.imread('/home/user/catkin_ws/src/cmput412-contest1/contest/src/test_cube.jpg',1)
        is_match, x = self.feature_detection(test_cube_img,20,30,1000)
        #print(x)
        if x < 200:
            return 'left'
        elif x > 400:
            return 'right'
        else:
            return 'center'

    #Takes picture of cube and saves it as a class attribute and saves it as a file for testing
    def take_picture_of_cube(self):
        self.cube_image = np.copy(self.cv_image)

        #save to file for testing
        cv2.imwrite('/home/user/catkin_ws/src/cmput412-contest1/contest/src/test_cube.jpg', self.cube_image)
        cv2.imshow('image',self.cube_image)
        cv2.waitKey(0)

    #TODO implement this
    def filter_colors(self, img, color):
        return False

    #Returns true if it has detected the wanted person in the picture
    #only returns true if it has detected it a certian number of times
    def detect_wanted(self):
        wanted_img = cv2.imread('/home/user/catkin_ws/src/cmput412-contest1/contest/src/wanted_cropped.jpg',1)
        detections = 0
        for i in range(0,10):
            is_match, x = self.feature_detection(wanted_img,390,30)
            if is_match:
                detections += 1
        return detections > 5



    #returns true if it detected the object. num_matches is number of matches BF feature detection uses.
    #heat is how sensitive the detection is, higher is less sensitive, lower means more sensitive but more false positives
    #num_features is how many features the ORB algorithm uses. 
    def feature_detection(self, ref_img, num_matches=390, heat=30, num_features=2000):    

        image_1 = ref_img
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
        orb = cv2.ORB_create(nfeatures = num_features)

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
        
            
        good_matches = matches[:num_matches] 
        

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

        #use this number, it is around 50 when wanted person is seen.
        #robot is ~3 meters away
        #print(np.sum(mask))
        match_number = np.sum(mask)
        #print(mask)
        #print("end of m")
        #length of mask when hit is 137 
        #when not detected its around 70

        #if (match_number > heat):
            #print("detected wanted person")
        #else:
            #print("not detected")

        #Catch the width and height from the main image
        h,w = gray_1.shape[:2]

        #Create a floating matrix for the new perspective
        pts = np.float32([[0,0],[0,h-1],[w-1,h-1],[w-1,0] ]).reshape(-1,1,2)

        #Create the perspective in the result 
        dst = cv2.perspectiveTransform(pts,M)

        #Draw the matching lines 
        

        # Draw the points of the new perspective in the result image (This is considered the bounding box)
        result = cv2.polylines(image_2, [np.int32(dst)], True, (50,0,255),3, cv2.LINE_AA)


        new_dst = np.int32(dst)

        x_avg = 0
        for i in range(4):
            x_avg += np.int32(new_dst)[i][0][0]
        x_avg = x_avg / 4




        #print(x_avg)
        #print('out of 700')
        cv2.imshow('image',img_2)
        cv2.imshow('Points',preview_1)
        
        cv2.imshow('Detection',image_2)       
           


        #uncomment this to see it visually
        #cv2.waitKey(1)
        return (match_number > heat), x_avg


#TODO implement proper message so it can call different functions in the Camera_utilities class
def server_callback(request):
    print("my callback has been called")
    #depending on what is in the message, do a certian thing

    return EmptyResponse()

rospy.init_node('camera_service_server')
#my_service = rospy.Service('/camera_functions', Empty, server_callback)
c_util = Camera_utilities()


rospy.spin()