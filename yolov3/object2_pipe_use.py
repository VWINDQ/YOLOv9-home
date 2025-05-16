#!/usr/bin/env python
import rospy
import torch
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
from gtts import gTTS
from playsound import playsound
import os
import warnings
import time

# Suppress FutureWarnings
warnings.simplefilter(action='ignore', category=FutureWarning)

# Initialize MediaPipe Hands and Drawing modules
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils

class YoloV5Ros:
    def __init__(self):
        # Initialize ROS node and publisher
        rospy.init_node('yolov5_ros_node', anonymous=True)
        self.publisher = rospy.Publisher('Test', String, queue_size=10)
        
        self.device = 'cuda' if torch.cuda.is_available() else 'cpu'
        self.model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/user/yolov5/runs/train/exp10/weights/best.pt').to(self.device)
        self.bridge = CvBridge()  # Initialize CvBridge for converting ROS images
        self.hands = mp_hands.Hands(min_detection_confidence=0.7, min_tracking_confidence=0.7)
        self.selected_object = None
        self.frame_id = 0  # Frame counter to skip frames
        self.last_detection_time = 0  # Time of last detection
        self.delay = 10  # Delay in seconds

        # Subscriber for Kinect RGB image
        rospy.Subscriber('/camera/rgb/image_raw', Image, self.image_callback)

    def image_callback(self, msg):
        try:
            # Convert the ROS image message to a format OpenCV can work with
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.process_frame(frame)
        except CvBridgeError as e:
            rospy.logerr(f'CvBridge Error: {e}')

    def process_frame(self, frame):
        current_time = time.time()
        self.frame_id += 1
        if self.frame_id % 2 != 0:  # Skip every other frame
            return

        # Increase resolution for better detection
        frame = cv2.resize(frame, (800, 600))

        # YOLOv5 Object Detection
        results = self.model(frame)

        # Filter detections based on confidence manually
        results.xyxy[0] = results.xyxy[0][results.xyxy[0][:, 4] >= 0.6]

        # MediaPipe Hand Detection
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results_hand = self.hands.process(frame_rgb)

        finger_tip_coords = None

        if results_hand.multi_hand_landmarks:
            for hand_landmarks in results_hand.multi_hand_landmarks:
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                
                # Get the tip of the index finger
                finger_tip = hand_landmarks.landmark[mp_hands.HandLandmark.INDEX_FINGER_TIP]
                finger_tip_coords = (int(finger_tip.x * 800), int(finger_tip.y * 600))
                
                # Draw a small circle at the fingertip
                cv2.circle(frame, finger_tip_coords, 10, (0, 255, 0), -1)  # Green dot for fingertip

        # Initialize variables for tracking selected object
        self.selected_object = None
        selected_bbox = None

        # Loop over detected objects
        for *xyxy, conf, cls in results.xyxy[0]:
            x1, y1, x2, y2 = map(int, xyxy)
            label = f'{self.model.names[int(cls)]} {conf:.2f}'
            color = (0, 255, 0)  # Green for unselected objects
            
            if finger_tip_coords and x1 < finger_tip_coords[0] < x2 and y1 < finger_tip_coords[1] < y2:
                color = (0, 0, 255)  # Red for selected object
                self.selected_object = label.split()[0]
                selected_bbox = (x1, y1, x2, y2)

            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        # Determine if selected object is left, right, or unknown
        if self.selected_object and finger_tip_coords and selected_bbox:
            # Check if enough time has passed since last detection
            if current_time - self.last_detection_time >= self.delay:
                x1, y1, x2, y2 = selected_bbox
                avg_bbox_x = (x1 + x2) // 2
                if avg_bbox_x < finger_tip_coords[0]:
                    position = 'left'
                elif avg_bbox_x > finger_tip_coords[0]:
                    position = 'right'
                else:
                    position = 'unknown'
                
                message = f"Please bring a bag on the {position} place on my hand!"
                cv2.putText(frame, message, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)

                # Print left or right to the terminal
                print(message)

                # Generate and play the speech command
                self.speak(message)

                # Publish "B" to the "Test" topic
                self.publish_message("B")

                # Update last detection time
                self.last_detection_time = current_time

                # Crop the selected object
                cropped_object = self.crop_object(frame, selected_bbox)
                
                # Display the cropped object
                self.display_cropped_object(cropped_object)
                
                # Save the cropped object image
                self.save_image(cropped_object, "selected_object.jpg")
        
        cv2.imshow("YOLOv5 Object Detection", frame)

    def speak(self, text):
        """Convert text to speech and play it."""
        tts = gTTS(text=text, lang='en')
        tts.save("output.mp3")
        playsound("output.mp3")
        os.remove("output.mp3")  # Remove the file after playing

    def publish_message(self, message):
        """Publish a message to the ROS topic."""
        self.publisher.publish(message)

    def crop_object(self, frame, bbox):
        """Crop the selected object from the frame."""
        x1, y1, x2, y2 = bbox
        cropped_object = frame[y1:y2, x1:x2]
        return cropped_object

    def display_cropped_object(self, cropped_object):
        """Display the cropped object."""
        cv2.imshow("Cropped Object", cropped_object)

    def save_image(self, image, filename):
        """Save the image to a file."""
        cv2.imwrite(filename, image)

    def run(self):
        rospy.spin()  # Keep the node running

if __name__ == '__main__':
    try:
        yolo = YoloV5Ros()
        yolo.run()
    except KeyboardInterrupt:
        pass

