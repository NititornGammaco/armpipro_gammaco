#!/usr/bin/python3

import time
import rospy
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from std_srvs.srv import Trigger
from color_tracking.srv import SetTarget
from visual_processing.msg import Result
from chassis_control.msg import SetVelocity

STOP_DIST_Y = 360
CENTER_X_LEFT = 265
CENTER_X_RIGHT = 295

find_block = False 
Pick_block = False

def stop():
    # Stop the robot by publishing zero velocity
    set_velocity.publish(0, 0, 0)
    try:
        # Try to call /color_tracking/exit service for clean shutdown
        rospy.wait_for_service('/color_tracking/exit', timeout=3)
        exit_srv = rospy.ServiceProxy('/color_tracking/exit', Trigger)
        resp = exit_srv()
        print("Called /color_tracking/exit, response:", resp)
    except Exception as e:
        print("Failed to call /color_tracking/exit:", e)

def result_callback(msg):
    global find_block , Pick_block
    print("Detected block center_x: {}, center_y: {}, radius: {}".format(msg.center_x, msg.center_y, msg.data))

    if find_block == False:
        if msg.center_y >= STOP_DIST_Y:
            if CENTER_X_LEFT <= msg.center_x <= CENTER_X_RIGHT:
                print("Block is centered and close. Stop and grab!")
                set_velocity.publish(0, 90, 0)
                rospy.sleep(1)
                find_block = True 
            elif msg.center_x < CENTER_X_LEFT:
                print("Block left. Turning left to center...")
                set_velocity.publish(15, 180, 0)
            elif msg.center_x > CENTER_X_RIGHT:
                print("Block right. Turning right to center...")
                set_velocity.publish(15, 0, 0) 
        else:
            if CENTER_X_LEFT <= msg.center_x <= CENTER_X_RIGHT:
                print("Block in center, drive straight.")
                set_velocity.publish(30, 90, 0)
            elif msg.center_x < CENTER_X_LEFT:
                print("Block left, turn left while moving forward.")
                set_velocity.publish(30, 180, 0)
            elif msg.center_x > CENTER_X_RIGHT:
                print("Block right, turn right while moving forward.")
                set_velocity.publish(15, 0, 0)
            else:
                set_velocity.publish(15, 90, 0)
    if find_block == True and Pick_block == False:
        bus_servo_control.set_servos(joints_pub, 2000, (
                (3, 220),
                (4, 760),
                (5, 280),
                (6, 500)
            )
        )
        rospy.sleep(2.5)
        bus_servo_control.set_servos(joints_pub, 2000, ((1, 480),))
        rospy.sleep(2)
        bus_servo_control.set_servos(joints_pub, 2000, (
                (3, 270),
                (4, 1000),
                (5, 840),
                (6, 500)
            )
        )
        rospy.sleep(2.5)
        Pick_block = True

if __name__ == '__main__':
    rospy.init_node('find_red_block')
    rospy.on_shutdown(stop)
    joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur', MultiRawIdPosDur, queue_size=1)
    rospy.sleep(0.2)
    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)

    rospy.wait_for_service('/color_tracking/enter')
    enter_srv = rospy.ServiceProxy('/color_tracking/enter', Trigger)
    enter_srv()

    bus_servo_control.set_servos(joints_pub, 2000, (
            (1, 100),
            (3, 270),
            (4, 1000),
            (5, 840),
            (6, 500)
        )
    )
    rospy.sleep(2.5)
    bus_servo_control.set_servos(joints_pub, 2000, (
            (3, 170),
            (4, 1000),
            (5, 750),
            (6, 500)
        )
    )
    rospy.sleep(3.0)

    rospy.wait_for_service('/color_tracking/set_target')
    set_target_srv = rospy.ServiceProxy('/color_tracking/set_target', SetTarget)
    set_target_srv("red")

    rospy.Subscriber("/visual_processing/result", Result, result_callback)
    rospy.spin()
