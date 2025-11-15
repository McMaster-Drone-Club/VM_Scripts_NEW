#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def callback(msg):
    string_data = msg.data
    print(string_data)

def subscriber():
    rospy.init_node('exampleNode2',anonymous=False)
    rospy.Subscriber('exampleTopic',String, callback)
    rospy.spin()


if __name__=='__main__':
    subscriber()
