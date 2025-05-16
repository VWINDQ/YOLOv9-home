#!/usr/bin/env python3

import rospy
import torch
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# โหลดโมเดล YOLOv5
model = torch.hub.load('/home/user/yolov5', 'custom', path='/home/user/Yolov9/src/yolov3/best.pt', source='local')
bridge = CvBridge()

def image_callback(msg):
    # แปลง ROS Image message เป็น OpenCV image
    frame = bridge.imgmsg_to_cv2(msg, "bgr8")

    # ตรวจจับถุงในภาพ
    detected, frame_with_detections = detect_bag(frame)

    if detected:
        rospy.loginfo("Bag detected!")
        # คุณสามารถทำการส่งสัญญาณหรือคำสั่งอื่น ๆ เมื่อพบถุง

    # แสดงภาพพร้อมกรอบการตรวจจับ
    cv2.imshow("YOLOv5 Detection", frame_with_detections)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        rospy.signal_shutdown("User requested shutdown")

def detect_bag(frame):
    """ฟังก์ชันในการตรวจจับถุงโดยใช้ YOLOv5"""
    results = model(frame)
    labels, coords = results.xyxyn[0][:, -1], results.xyxyn[0][:, :-1]

    n = len(labels)
    for i in range(n):
        row = coords[i]
        if row[4] >= 0.5:  # ตรวจสอบความมั่นใจ (confidence threshold)
            x1, y1, x2, y2 = int(row[0] * frame.shape[1]), int(row[1] * frame.shape[0]), int(row[2] * frame.shape[1]), int(row[3] * frame.shape[0])
            cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(frame, f'{int(labels[i])}: {row[4]:.2f}', (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
            return True, frame

    return False, frame

def main():
    rospy.init_node('bag_detector')
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

    rospy.loginfo("YOLOv5 Bag Detector is running...")
    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

