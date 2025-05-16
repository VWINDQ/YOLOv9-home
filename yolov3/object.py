#!/usr/bin/env python
import rospy
import cv2
import torch
import time
import warnings
from std_msgs.msg import String

# Suppress FutureWarnings
warnings.simplefilter(action='ignore', category=FutureWarning)

def detect_and_display():
    # Initialize the YOLOv5 model
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/user/yolov5/runs/train/exp7/weights/best.pt')
    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 10)

    # Create a publisher to send messages to the 'Test' topic
    pub = rospy.Publisher('Test', String, queue_size=10)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            rospy.logerr("Failed to capture image")
            continue

        # Perform detection on the frame
        results = model(frame)
        for *xyxy, conf, cls in results.xyxy[0]:
            trust = conf * 100
            if trust > 60:
                rospy.loginfo("Detected bag")
                # Send message 'B' to the 'Test' topic
                pub.publish("B")

            # Draw bounding box and label on the frame
            label = f'{model.names[int(cls)]} {conf:.2f}'
            cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (255, 0, 0), 2)
            cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

        # Display the frame
        cv2.imshow("YOLOv5 Object Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        time.sleep(1)

    # Release resources
    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    # Initialize ROS node
    rospy.init_node('yolo_detection_node')
    detect_and_display()

