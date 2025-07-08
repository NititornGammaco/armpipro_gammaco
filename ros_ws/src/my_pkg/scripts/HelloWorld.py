#!/usr/bin/env python3
import rospy
from std_msgs.msg import String


pub = rospy.Publisher('chatter',String,queue_size = 10)
rospy.init_node('talker')
rate = rospy.Rate(1)
count = 0
while (not rospy.is_shutdown()):
	text = "Hello %s" % count
	rospy.loginfo(text)
	pub.publish(text)
	count = count + 1
	rate.sleep()
