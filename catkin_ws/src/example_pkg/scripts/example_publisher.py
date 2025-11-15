#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def publisher():
    pub = rospy.Publisher('exampleTopic', String, queue_size=10)

    rospy.init_node('exampleNode',anonymous=False)

    rate = rospy.Rate(10) #Hz

    while not rospy.is_shutdown():
        string_data = "Hello Drone Dojo! What is up Bro Bro?"
        pub.publish(string_data)
        rate.sleep()

if __name__=='__main__':
    publisher()
