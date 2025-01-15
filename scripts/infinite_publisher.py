#! /usr/bin/env python

import rospy
from std_msgs.msg import Int16

pub = rospy.Publisher('hello', Int16, queue_size=1)

rospy.init_node('hello')
while not rospy.is_shutdown():
    pub.publish(69)
    print("published")