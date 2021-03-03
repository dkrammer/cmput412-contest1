#! /usr/bin/env python

#import rospkg
import rospy
#import sys
import time
from contest_class import ContestClass

def main():

    rospy.init_node('contest_node', anonymous=True)
    contest_obj = ContestClass()
    time.sleep(0.1)
    
    contest_obj.task_one()
    contest_obj.task_two()
    contest_obj.task_three()
    contest_obj.task_four()

    rospy.loginfo("Loop finished. Goodbye!")


main()