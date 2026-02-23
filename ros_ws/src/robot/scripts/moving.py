#!/usr/bin/env python3

import rospy                                   # Import ROS Python library
from chassis_control.msg import *              # Import message type for controlling the robot chassis

start = True                                   # Flag variable used to control the main loop

def stop():
    global start
    start = False                              # Stop the loop when shutdown is triggered
    set_velocity.publish(0,0,0)                # Send stop command (zero velocity)

rospy.init_node('robot_moving')                # Create ROS node named 'robot_moving'
rospy.on_shutdown(stop)                        # Register stop() to run when ROS shuts down

set_velocity = rospy.Publisher(
    '/chassis_control/set_velocity',           # Topic for sending velocity commands
    SetVelocity,                               # Message type used for velocity control
    queue_size=1                               # Keep only the latest command in the queue
)

rospy.sleep(1)                                 # Wait 1 second to ensure publisher is ready

while start:
    set_velocity.publish(30,90,0)              # Move forward with speed 30 (direction 90°)
    rospy.sleep(3)                             # Move for 3 seconds

    set_velocity.publish(30,0,0)               # Move right with speed 30 (direction 0°)
    rospy.sleep(3)                             # Move for 3 seconds

    set_velocity.publish(30,180,0)             # Move left with speed 30 (direction 180°)
    rospy.sleep(3)                             # Move for 3 seconds

    set_velocity.publish(30,270,0)             # Move backward with speed 30 (direction 270°)
    rospy.sleep(3)                             # Move for 3 seconds
