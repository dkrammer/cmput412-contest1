#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2



class Sobel(object):

    def __init__(self):
    
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw",Image,self.camera_callback)
        self.bridge_object = CvBridge()

    def camera_callback(self,data):
        try:
            # We select bgr8 because its the OpenCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
        except CvBridgeError as e:
            print(e)

        cv2.imshow('image', cv_image)
        cv2.waitKey(1)

        #drone_image = cv2.imwrite('drone_image.jpg',cv_image)
        
        #cv2.imshow('original',img)
        drone_img = cv2.imwrite('/home/user/catkin_ws/src/cmput412-contest1/contest/src/wanted.jpg', cv_image)


        cv2.waitKey(0)





def main():
    load_image_object = Sobel()
    rospy.init_node('sobel_node', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
