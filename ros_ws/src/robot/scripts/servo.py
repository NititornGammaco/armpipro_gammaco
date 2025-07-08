#!/usr/bin/python3

import time
import rospy
from armpi_pro import bus_servo_control
from hiwonder_servo_msgs.msg import MultiRawIdPosDur

rospy.init_node('servo_test')
joints_pub = rospy.Publisher('/servo_controllers/port_id_1/multi_id_pos_dur',MultiRawIdPosDur, queue_size=1)
rospy.sleep(0.2)

bus_servo_control.set_servos(joints_pub, 2000, (
        (1, 100),
        (2, 500),
        (3, 270),
        (4, 1000),
        (5, 840),
        (6, 500)
    )
)
rospy.sleep(2.0)
bus_servo_control.set_servos(joints_pub, 2000, (
        (1, 100),
        (2, 500),
        (3, 200),
        (4, 1000),
        (5, 720),
        (6, 500)
    )
)
