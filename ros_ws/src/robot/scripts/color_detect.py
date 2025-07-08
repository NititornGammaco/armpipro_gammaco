#!/usr/bin/python3
import rospy
from std_srvs.srv import Trigger
from color_tracking.srv import SetTarget
from visual_processing.msg import Result

def stop():
    rospy.wait_for_service('/color_tracking/exit', timeout=3)
    exit_srv = rospy.ServiceProxy('/color_tracking/exit', Trigger)
    resp = exit_srv()
    print("Called /color_tracking/exit, response:", resp)

def result_callback(msg):
    print("Detected block center_x: {}, center_y: {}, radius: {}".format(msg.center_x, msg.center_y, msg.data))

rospy.init_node('color_detect')
rospy.on_shutdown(stop)

rospy.wait_for_service('/color_tracking/enter')
enter_srv = rospy.ServiceProxy('/color_tracking/enter', Trigger)
enter_srv()

rospy.wait_for_service('/color_tracking/set_target')
set_target_srv = rospy.ServiceProxy('/color_tracking/set_target', SetTarget)
set_target_srv("red")

rospy.Subscriber("/visual_processing/result", Result, result_callback)
rospy.spin()
