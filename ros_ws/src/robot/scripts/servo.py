#!/usr/bin/env python3

import time                                             # ใช้งานโมดูล time สำหรับหน่วงเวลา
import rospy                                            # ใช้งาน ROS กับ Python
from armpi_pro import bus_servo_control                 # นำเข้าโมดูลควบคุมเซอร์โวแบบบัส
from hiwonder_servo_msgs.msg import MultiRawIdPosDur    # นำเข้า message สำหรับควบคุมเซอร์โวหลายตัว

rospy.init_node('servo_test')                           # สร้าง ROS node ชื่อ 'servo_test'
joints_pub = rospy.Publisher(
    '/servo_controllers/port_id_1/multi_id_pos_dur',    # สร้าง publisher ไปที่ topic สำหรับควบคุมเซอร์โว
    MultiRawIdPosDur,                                   # ใช้ message type MultiRawIdPosDur
    queue_size=1                                        # กำหนดคิวแค่ 1 เพื่อรับเฉพาะคำสั่งล่าสุด
)
rospy.sleep(0.2)                                        # หน่วงเวลา 0.2 วินาที ให้ publisher พร้อม

bus_servo_control.set_servos(joints_pub, 2000, (        # สั่งให้เซอร์โวทุกตัวไปยังตำแหน่งที่กำหนดใน 2 วินาที
        (1, 100),                                       # เซอร์โว ID 1 ไปที่ตำแหน่ง 100
        (2, 500),                                       # เซอร์โว ID 2 ไปที่ตำแหน่ง 500
        (3, 270),                                       # เซอร์โว ID 3 ไปที่ตำแหน่ง 270
        (4, 1000),                                      # เซอร์โว ID 4 ไปที่ตำแหน่ง 1000
        (5, 840),                                       # เซอร์โว ID 5 ไปที่ตำแหน่ง 840
        (6, 500)                                        # เซอร์โว ID 6 ไปที่ตำแหน่ง 500
    )
)
rospy.sleep(2.0)                                        # รอ 2 วินาที ให้เซอร์โวเคลื่อนที่จบ

bus_servo_control.set_servos(joints_pub, 2000, (        # สั่งให้เซอร์โวขยับไปตำแหน่งใหม่ใน 2 วินาที
        (1, 100),                                       # เซอร์โว ID 1 ไปที่ตำแหน่ง 100
        (2, 500),                                       # เซอร์โว ID 2 ไปที่ตำแหน่ง 500
        (3, 200),                                       # เซอร์โว ID 3 ไปที่ตำแหน่ง 200 (เปลี่ยนจากเดิม)
        (4, 1000),                                      # เซอร์โว ID 4 ไปที่ตำแหน่ง 1000
        (5, 720),                                       # เซอร์โว ID 5 ไปที่ตำแหน่ง 720 (เปลี่ยนจากเดิม)
        (6, 500)                                        # เซอร์โว ID 6 ไปที่ตำแหน่ง 500
    )
)
