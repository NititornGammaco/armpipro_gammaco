#!/usr/bin/python3

import rospy
from std_srvs.srv import Trigger, SetBool
from visual_processing.msg import Result
from chassis_control.msg import SetVelocity

# ===================== SETTINGS =====================
MOVE_SPEED = 30
MOVE_TIME = 2.0

DIR_RIGHT = 0
DIR_FORWARD = 90
DIR_LEFT = 180
DIR_BACKWARD = 270

# ===================== VARIABLES =====================
current_tag = 0
robot_busy = False

# ===================== SHUTDOWN =====================
def stop():
    set_velocity.publish(0, 0, 0)
    rospy.wait_for_service('/apriltag_detect/exit', timeout=2)
    exit_srv = rospy.ServiceProxy('/apriltag_detect/exit', Trigger)
    exit_srv()

# ===================== APRILTAG CALLBACK =====================
def result_callback(msg):
    global current_tag, robot_busy

    tag_id = int(msg.data)

    print("Tag:", tag_id,
          "X:", msg.center_x,
          "Y:", msg.center_y)

    if robot_busy or current_tag != 0:
        return

    current_tag = tag_id
    print("Command Tag:", tag_id)

# ===================== MOVE ROBOT =====================
def move_robot(tag_id):
    global robot_busy

    robot_busy = True

    if tag_id == 1:
        print("Tag 1 -> FORWARD")
        set_velocity.publish(MOVE_SPEED, DIR_FORWARD, 0)

    elif tag_id == 2:
        print("Tag 2 -> BACKWARD")
        set_velocity.publish(MOVE_SPEED, DIR_BACKWARD, 0)

    elif tag_id == 3:
        print("Tag 3 -> RIGHT")
        set_velocity.publish(MOVE_SPEED, DIR_RIGHT, 0)

    elif tag_id == 4:
        print("Tag 4 -> LEFT")
        set_velocity.publish(MOVE_SPEED, DIR_LEFT, 0)

    rospy.sleep(MOVE_TIME)
    set_velocity.publish(0, 0, 0)

    print("STOP")
    robot_busy = False

# ===================== MAIN =====================
if __name__ == '__main__':
    rospy.init_node('apriltag_robot_moving')
    rospy.on_shutdown(stop)

    set_velocity = rospy.Publisher('/chassis_control/set_velocity', SetVelocity, queue_size=1)
    rospy.sleep(1)

    # Start AprilTag detection
    rospy.wait_for_service('/apriltag_detect/enter')
    enter_srv = rospy.ServiceProxy('/apriltag_detect/enter', Trigger)
    enter_srv()

    # Enable AprilTag detection
    rospy.wait_for_service('/apriltag_detect/set_running')
    set_running_srv = rospy.ServiceProxy('/apriltag_detect/set_running', SetBool)
    set_running_srv(True)

    # Read AprilTag result
    rospy.Subscriber('/visual_processing/result', Result, result_callback)

    print("AprilTag Robot Ready")
    print("Tag 1 = Forward")
    print("Tag 2 = Backward")
    print("Tag 3 = Right")
    print("Tag 4 = Left")

    rate = rospy.Rate(20)

    while not rospy.is_shutdown():
        if current_tag != 0 and not robot_busy:
            tag_to_run = current_tag
            current_tag = 0
            move_robot(tag_to_run)

        rate.sleep()
