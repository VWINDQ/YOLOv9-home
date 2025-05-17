#!/usr/bin/env python3

import rospy
import cv2
import mediapipe as mp
from geometry_msgs.msg import Twist
from std_msgs.msg import String

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
hands = mp_hands.Hands()
mp_drawing = mp.solutions.drawing_utils

def detect_hand_gesture(hand_landmarks):
    """
    ฟังก์ชันตรวจจับการชูนิ้วเพื่อระบุทิศทางหรือคำสั่งที่ต้องการ
    """
    fingers = []

    # Thumb: เช็คว่าข้อปลายนิ้วโป้งอยู่ทางขวาของข้อฐานนิ้วหรือไม่ (สำหรับมือขวา)
    if hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_TIP].x > hand_landmarks.landmark[mp_hands.HandLandmark.THUMB_IP].x:
        fingers.append(1)
    else:
        fingers.append(0)

    # Fingers: เช็คว่าปลายนิ้วอยู่สูงกว่าข้อกลางหรือไม่
    for tip_index in [mp_hands.HandLandmark.INDEX_FINGER_TIP,
                      mp_hands.HandLandmark.MIDDLE_FINGER_TIP,
                      mp_hands.HandLandmark.RING_FINGER_TIP,
                      mp_hands.HandLandmark.PINKY_TIP]:
        if hand_landmarks.landmark[tip_index].y < hand_landmarks.landmark[tip_index - 2].y:
            fingers.append(1)
        else:
            fingers.append(0)

    # นับจำนวนของนิ้วที่ชูขึ้น
    finger_count = fingers.count(1)

    # ตรวจสอบการชูนิ้วเพื่อระบุคำสั่ง
    if finger_count == 1:
        return "forward"
    elif finger_count == 2:
        return "left"
    elif finger_count == 3:
        return "right"
    elif finger_count == 4:
        return "backward"
    elif finger_count == 5:
        return "stop"
    else:
        return "none"

def carry_my_luggage_node():
    rospy.init_node('carry_my_luggage_node', anonymous=True)
    pub_cmd_vel = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    pub_hand_gesture = rospy.Publisher('/hand_gesture', String, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = hands.process(image)
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        command = "none"

        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                mp_drawing.draw_landmarks(image, hand_landmarks, mp_hands.HAND_CONNECTIONS)

                # ตรวจจับท่าทางของมือ
                command = detect_hand_gesture(hand_landmarks)
                rospy.loginfo(f"Detected command: {command}")
                pub_hand_gesture.publish(command)

                # ส่งคำสั่งไปยังหุ่นยนต์
                twist = Twist()
                if command == "forward":
                    twist.linear.x = 0.2
                elif command == "left":
                    twist.angular.z = 0.5
                elif command == "right":
                    twist.angular.z = -0.5
                elif command == "backward":
                    twist.linear.x = -0.2
                elif command == "stop":
                    twist.linear.x = 0.0
                    twist.angular.z = 0.0

                pub_cmd_vel.publish(twist)

        # แสดงผลภาพพร้อมคำสั่ง
        cv2.putText(image, f"Command: {command}", (50, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 2, cv2.LINE_AA)

        cv2.imshow('Carry My Luggage', image)
        if cv2.waitKey(5) & 0xFF == 27:  # กด 'Esc' เพื่อออก
            break

        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        carry_my_luggage_node()
    except rospy.ROSInterruptException:
        pass

