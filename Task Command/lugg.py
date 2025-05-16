#!/usr/bin/env python3

import cv2
import mediapipe as mp
import rospy
import torch
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import serial
import time
import numpy as np

class BagPickerBot:
    def __init__(self):
        # YOLO model สำหรับตรวจจับถุง
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/user/Downloads/additional_ws/src/yolov3/best.pt')

        # ROS Nodes
        rospy.init_node('bag_picker_bot')
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.bridge = CvBridge()

        # MediaPipe Hands
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(max_num_hands=1)
        self.mp_drawing = mp.solutions.drawing_utils

        # ตัวแปรสำคัญ
        self.left_bag = None
        self.right_bag = None
        self.selected_bag = None
        self.ser = serial.Serial('/dev/ttyUSB0', 9600)  # เชื่อมต่อ Arduino
        self.kinect_frame_width = 640  # กำหนดความกว้างของภาพจาก Kinect

        # Subscribe to Kinect camera
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        # แปลง ROS Image message เป็น OpenCV image
        img = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        # ตรวจจับวัตถุด้วย YOLO
        results = self.model(img)
        # แยกถุงซ้ายและขวา
        self.left_bag, self.right_bag = self.identify_bags(results)

    def identify_bags(self, results):
        left_bag = None
        right_bag = None
        
        for result in results.xyxy[0]:  # ดึง bounding box ที่ตรวจพบ
            x1, y1, x2, y2, conf, cls = result
            if left_bag is None or x1 < left_bag[0]:
                right_bag = left_bag
                left_bag = (x1, y1, x2, y2)
            elif right_bag is None or x1 > right_bag[0]:
                right_bag = (x1, y1, x2, y2)
        
        return left_bag, right_bag

    def detect_finger_direction(self, frame):
        # แปลงภาพ BGR เป็น RGB
        rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # ตรวจจับมือ
        results = self.hands.process(rgb_image)

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                # วาด landmarks บนภาพ
                self.mp_drawing.draw_landmarks(frame, hand_landmarks, self.mp_hands.HAND_CONNECTIONS)

                # ดึงพิกัดของนิ้วชี้ (index finger)
                index_finger_tip = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_TIP]
                index_finger_mcp = hand_landmarks.landmark[self.mp_hands.HandLandmark.INDEX_FINGER_MCP]

                # คำนวณทิศทางการชี้นิ้ว
                direction = np.array([index_finger_tip.x - index_finger_mcp.x,
                                      index_finger_tip.y - index_finger_mcp.y,
                                      index_finger_tip.z - index_finger_mcp.z])

                # เช็คทิศทางว่าชี้ไปทางซ้ายหรือขวา (แกน x)
                if direction[0] > 0:
                    return "right"
                else:
                    return "left"
        return None

    def get_user_input(self, frame):
        # ตรวจจับทิศทางการชี้นิ้วจาก frame
        finger_direction = self.detect_finger_direction(frame)

        if finger_direction == "left":
            self.selected_bag = self.left_bag
            rospy.loginfo("User selected the left bag.")
        elif finger_direction == "right":
            self.selected_bag = self.right_bag
            rospy.loginfo("User selected the right bag.")
        else:
            rospy.loginfo("No finger direction detected.")
    
    def move_towards_bag(self):
        if self.selected_bag is None:
            rospy.loginfo("No bag selected.")
            return

        x1, y1, x2, y2 = self.selected_bag
        bag_center_x = (x1 + x2) / 2
        cmd_vel = Twist()

        # หมุนตัวหุ่นยนต์ให้ถุงอยู่ตรงกลาง
        if bag_center_x < self.kinect_frame_width / 2 - 20:  # ถุงอยู่ทางซ้าย
            cmd_vel.angular.z = 0.1  # หมุนตัวไปทางซ้าย
        elif bag_center_x > self.kinect_frame_width / 2 + 20:  # ถุงอยู่ทางขวา
            cmd_vel.angular.z = -0.1  # หมุนตัวไปทางขวา
        else:
            # ถุงอยู่ตรงกลางแล้ว เคลื่อนที่ไปข้างหน้า
            cmd_vel.linear.x = 0.2

        self.cmd_vel_pub.publish(cmd_vel)

    def send_arm_command(self, command):
        self.ser.write(command.encode())  # ส่งคำสั่งไปยัง Arduino

    def pick_bag(self):
        # ส่งคำสั่งไปยังแขนกลเพื่อหยิบถุง
        self.send_arm_command("B")  # ยื่นแขน
        time.sleep(2)  # รอแขนเคลื่อนไหว
        self.send_arm_command("D")  # ขยับแขนไปตำแหน่งถุง
        time.sleep(2)
        self.send_arm_command("C")  # ปิด gripper เพื่อจับถุง
        time.sleep(2)
        self.send_arm_command("E")  # ดึงแขนกลับ
        time.sleep(2)

    def run(self):
        cap = cv2.VideoCapture(0)  # เปิดกล้อง webcam เพื่อจับการชี้นิ้ว

        while cap.isOpened() and not rospy.is_shutdown():
            success, frame = cap.read()
            if not success:
                break
            
            # ตรวจจับและแสดงการชี้นิ้ว
            self.get_user_input(frame)

            # แสดงผลภาพที่จับได้จาก webcam
            cv2.imshow('Hand Detection', frame)
            if cv2.waitKey(5) & 0xFF == 27:  # กด ESC เพื่อออก
                break
            
            # เคลื่อนที่ไปยังถุงที่ผู้ใช้เลือก
            if self.selected_bag:
                self.move_towards_bag()

                # ตรวจสอบว่าถึงถุงหรือยัง ถ้าใช่เริ่มการหยิบถุง
                if self.is_near_bag():
                    rospy.loginfo("Reached the bag. Picking up...")
                    self.pick_bag()
                    break

        cap.release()
        cv2.destroyAllWindows()

    def is_near_bag(self):
        # คุณสามารถเพิ่มการตรวจสอบระยะทางจาก Depth data ได้ในฟังก์ชันนี้
        # เพื่อให้หุ่นรู้ว่าถึงตำแหน่งที่ใกล้ถุงพอที่จะหยิบแล้ว
        return True  # ตัวอย่าง: คุณอาจเพิ่มการตรวจสอบระยะที่นี่


if __name__ == '__main__':
    try:
        bot = BagPickerBot()
        bot.run()
    except rospy.ROSInterruptException:
        pass

