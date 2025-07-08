#!/usr/bin/env python3
import rospy
from std_msgs.msg import String

def callback(data):
	rospy.loginfo(data.data)


while (not rospy.is_shutdown()):
	rospy.Subscriber('chatter',String,callback)
	rospy.init_node('litsener')
	rospy.spin()
