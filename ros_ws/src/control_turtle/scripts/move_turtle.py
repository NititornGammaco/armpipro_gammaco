#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

rospy.init_node('control')
pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
rate = rospy.Rate(10) # 10hz

move_cmd = Twist()
move_cmd.linear.x = 5.0
move_cmd.angular.z = 2.0

while not rospy.is_shutdown():
    pub.publish(move_cmd)
    rate.sleep()
