#!/usr/bin/env python
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2

def detect_and_display():
    device = 'cuda' if torch.cuda.is_available() else 'cpu'
    model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/user/yolov5/runs/train/exp10/weights/best.pt').to(device)
    bridge = CvBridge()

    # Create a publisher for the "Test" topic
    pub = rospy.Publisher('Test', String, queue_size=10)

    def image_callback(msg):
        try:
            # Convert the ROS image message to a format OpenCV can work with
            frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            frame = cv2.resize(frame, (640, 480))  # Resize for faster processing

            results = model(frame)
            detected = False

            for *xyxy, conf, cls in results.xyxy[0]:
                trust = conf * 100
                if trust > 60:
                    detected = True
                    rospy.loginfo("Detected bag")

                label = f'{model.names[int(cls)]} {conf:.2f}'
                cv2.rectangle(frame, (int(xyxy[0]), int(xyxy[1])), (int(xyxy[2]), int(xyxy[3])), (255, 0, 0), 2)
                cv2.putText(frame, label, (int(xyxy[0]), int(xyxy[1]) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            if detected:
                # Publish message "B" to the "Test" topic
                pub.publish("B")

            # Display the frame
            cv2.imshow("YOLOv5 Object Detection", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                rospy.signal_shutdown('User requested shutdown')
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')

    rospy.init_node('yolo_detection_node')
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)

    rospy.spin()

    cv2.destroyAllWindows()

if __name__ == '__main__':
    detect_and_display()

