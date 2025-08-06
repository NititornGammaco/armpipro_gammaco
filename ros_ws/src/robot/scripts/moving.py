#!/usr/bin/env python3

import rospy                                   # ใช้งาน ROS กับ Python
from chassis_control.msg import *              # นำเข้า Message ที่ใช้ควบคุมแชสซีหุ่นยนต์

start = True                                   # กำหนด flag สำหรับควบคุม loop การทำงาน

def stop():
    global start
    start = False                              # เมื่อถูกเรียก จะหยุดการวน loop
    set_velocity.publish(0,0,0)                # ส่งคำสั่งหยุดหุ่นยนต์ (ความเร็ว 0)

rospy.init_node('robot_moving')                # สร้าง node ชื่อ 'robot_moving'
rospy.on_shutdown(stop)                        # ตั้ง callback ให้เรียก stop() เมื่อ ROS shutdown

set_velocity = rospy.Publisher(
    '/chassis_control/set_velocity',           # ประกาศ publisher ส่งข้อมูลไปยัง topic นี้
    SetVelocity,                               # ประเภท message ที่ใช้
    queue_size=1                               # กำหนดคิวแค่ 1 (เน้นสถานะล่าสุด)
)

rospy.sleep(1)                                 # รอ 1 วินาที ให้ publisher พร้อมก่อนเริ่มทำงาน

while start:
    set_velocity.publish(30,90,0)              # ส่งคำสั่งให้หุ่นยนต์เดินหน้า ด้วยความเร็ว 30
    rospy.sleep(3)                             # รอ 3 วินาที

    set_velocity.publish(30,0,0)               # ส่งคำสั่งให้หุ่นยนต์ไปทางขวา ด้วยความเร็ว 30
    rospy.sleep(3)                             # รอ 3 วินาที

    set_velocity.publish(30,180,0)             # ส่งคำสั่งให้หุ่นยนต์ไปทางซ้าย ด้วยความเร็ว 30
    rospy.sleep(3)                             # รอ 3 วินาที

    set_velocity.publish(30,270,0)             # ส่งคำสั่งให้หุ่นยนต์ถอยหลัง ด้วยความเร็ว 30
    rospy.sleep(3)                             # รอ 3 วินาที
