#!/usr/bin/python3

import time
import rospy
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

from std_srvs.srv import Trigger
from color_tracking.srv import SetTarget
from visual_processing.msg import Result
from chassis_control.msg import SetVelocity

# ===================== CONSTANT PARAMETERS =====================
STOP_DIST_Y = 360                 # Y-distance threshold to stop near the block
CENTER_X_LEFT = 265               # Left boundary of camera center zone
CENTER_X_RIGHT = 295              # Right boundary of camera center zone

# ===================== STATE FLAGS =====================
find_block = False                # True when block is aligned and close
Pick_block = False                # True when block has been picked

# ===================== SHUTDOWN HANDLER =====================
def stop():
    # Stop robot movement
    set_velocity.publish(0, 0, 0)

    try:
        # Call exit service to stop color tracking safely
        rospy.wait_for_service('/color_tracking/exit', timeout=3)
        exit_srv = rospy.ServiceProxy('/color_tracking/exit', Trigger)
        resp = exit_srv()
        print("Called /color_tracking/exit, response:", resp)
    except Exception as e:
        print("Failed to call /color_tracking/exit:", e)

# ===================== COLOR DETECTION CALLBACK =====================
def result_callback(msg):
    global find_block, Pick_block

    # Print detected object information
    print(
        "Detected block center_x: {}, center_y: {}, radius: {}".format(
            msg.center_x, msg.center_y, msg.data
        )
    )

    # ===================== BLOCK SEARCH & ALIGNMENT =====================
    if not find_block:
        # Block is close enough
        if msg.center_y >= STOP_DIST_Y:
            if CENTER_X_LEFT <= msg.center_x <= CENTER_X_RIGHT:
                # Block is centered and close
                print("Block is centered and close. Stop and grab!")
                set_velocity.publish(0, 90, 0)
                rospy.sleep(1)
                find_block = True

            elif msg.center_x < CENTER_X_LEFT:
                # Block is to the left
                print("Block left. Turning left to center...")
                set_velocity.publish(15, 180, 0)

            elif msg.center_x > CENTER_X_RIGHT:
                # Block is to the right
                print("Block right. Turning right to center...")
                set_velocity.publish(15, 0, 0)

        # Block is still far away
        else:
            if CENTER_X_LEFT <= msg.center_x <= CENTER_X_RIGHT:
                # Block centered, move forward
                print("Block in center, drive straight.")
                set_velocity.publish(30, 90, 0)

            elif msg.center_x < CENTER_X_LEFT:
                # Block left, move forward while turning left
                print("Block left, turn left while moving forward.")
                set_velocity.publish(30, 180, 0)

            elif msg.center_x > CENTER_X_RIGHT:
                # Block right, move forward while turning right
                print("Block right, turn right while moving forward.")
                set_velocity.publish(15, 0, 0)

            else:
                # Default slow forward motion
                set_velocity.publish(15, 90, 0)

    # ===================== PICK-UP SEQUENCE =====================
    if find_block and not Pick_block:
        # Move arm to grabbing position
        bus_servo_control.set_servos(joints_pub, 2000, (
            (3, 220),
            (4, 760),
            (5, 280),
            (6, 500)
        ))
        rospy.sleep(2.5)

        # Close gripper
        bus_servo_control.set_servos(joints_pub, 2000, ((1, 480),))
        rospy.sleep(2)

        # Lift the block
        bus_servo_control.set_servos(joints_pub, 2000, (
            (3, 270),
            (4, 1000),
            (5, 840),
            (6, 500)
        ))
        rospy.sleep(2.5)

        Pick_block = True            # Mark block as picked

# ===================== MAIN PROGRAM =====================
if __name__ == '__main__':
    rospy.init_node('find_red_block')           # Initialize ROS node
    rospy.on_shutdown(stop)                     # Register shutdown handler

    # Publisher for servo control
    joints_pub = rospy.Publisher(
        '/servo_controllers/port_id_1/multi_id_pos_dur',
        MultiRawIdPosDur,
        queue_size=1
    )
    rospy.sleep(0.2)

    # Publisher for chassis movement
    set_velocity = rospy.Publisher(
        '/chassis_control/set_velocity',
        SetVelocity,
        queue_size=1
    )

    # Start color tracking
    rospy.wait_for_service('/color_tracking/enter')
    enter_srv = rospy.ServiceProxy('/color_tracking/enter', Trigger)
    enter_srv()

    # Move arm to initial standby position
    bus_servo_control.set_servos(joints_pub, 2000, (
        (1, 100),
        (3, 270),
        (4, 1000),
        (5, 840),
        (6, 500)
    ))
    rospy.sleep(2.5)

    # Lower arm slightly for detection
    bus_servo_control.set_servos(joints_pub, 2000, (
        (3, 170),
        (4, 1000),
        (5, 750),
        (6, 500)
    ))
    rospy.sleep(3.0)

    # Set target color to red
    rospy.wait_for_service('/color_tracking/set_target')
    set_target_srv = rospy.ServiceProxy('/color_tracking/set_target', SetTarget)
    set_target_srv("red")

    # Subscribe to vision processing result
    rospy.Subscriber("/visual_processing/result", Result, result_callback)

    # Keep node running
    rospy.spin()
