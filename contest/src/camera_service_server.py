#! /usr/bin/env python

import rospy 
#TODO import a suitable message. 
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.

#TODO put in class/ functions that can do the things we need.


def server_callback(request):
    print("my callback has been called")
    #depending on what is in the message, do a certian thing

    return EmptyResponse()

rospy.init_node('camera_service_server')
my_service = rospy.Service('/camera_functions', Empty, server_callback)
rospy.spin()