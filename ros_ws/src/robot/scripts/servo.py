#!/usr/bin/env python3

import time                                             # Import time module for delays
import rospy                                            # Import ROS Python library
from armpi_pro import bus_servo_control                 # Import bus servo control module
from hiwonder_servo_msgs.msg import MultiRawIdPosDur    # Import message type for controlling multiple servos

rospy.init_node('servo_test')                           # Create a ROS node named 'servo_test'
joints_pub = rospy.Publisher(
    '/servo_controllers/port_id_1/multi_id_pos_dur',    # Create publisher for servo control topic
    MultiRawIdPosDur,                                   # Use MultiRawIdPosDur message type
    queue_size=1                                        # Keep only the latest command in the queue
)
rospy.sleep(0.2)                                        # Wait 0.2 seconds to ensure publisher is ready

bus_servo_control.set_servos(joints_pub, 2000, (        # Move all servos to specified positions in 2 seconds
        (1, 100),                                       # Servo ID 1 → position 100
        (2, 500),                                       # Servo ID 2 → position 500
        (3, 270),                                       # Servo ID 3 → position 270
        (4, 1000),                                      # Servo ID 4 → position 1000
        (5, 840),                                       # Servo ID 5 → position 840
        (6, 500)                                        # Servo ID 6 → position 500
    )
)
rospy.sleep(2.0)                                        # Wait 2 seconds for movement to complete

bus_servo_control.set_servos(joints_pub, 2000, (        # Move servos to new positions in 2 seconds
        (1, 100),                                       # Servo ID 1 → position 100
        (2, 500),                                       # Servo ID 2 → position 500
        (3, 200),                                       # Servo ID 3 → position 200 (changed)
        (4, 1000),                                      # Servo ID 4 → position 1000
        (5, 720),                                       # Servo ID 5 → position 720 (changed)
        (6, 500)                                        # Servo ID 6 → position 500
    )
)
rospy.sleep(2.0)                                        # Wait 2 seconds for movement to complete
