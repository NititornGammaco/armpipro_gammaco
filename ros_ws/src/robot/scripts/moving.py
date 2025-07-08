#!/usr/bin/python3

import rospy
from chassis_control.msg import *

start = True
def stop():
    global start

    start = False
    set_velocity.publish(0,0,0)

rospy.init_node('robot_moving')
rospy.on_shutdown(stop)
set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
rospy.sleep(1)

while start:
    set_velocity.publish(30,90,0)
    rospy.sleep(3)
    set_velocity.publish(30,0,0)
    rospy.sleep(3)
    set_velocity.publish(30,270,0)
    rospy.sleep(3)
    set_velocity.publish(30,180,0)
    rospy.sleep(3)
