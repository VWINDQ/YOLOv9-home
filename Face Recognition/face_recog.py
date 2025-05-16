#!/usr/bin/env python3

import rospy
import cv2
import torch
import face_recognition
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from sound_play.libsoundplay import SoundClient

# Initialize ROS node
rospy.init_node('guest_recognition')

# Load YOLOv5 model
model = torch.hub.load('ultralytics/yolov5', 'yolov5s')  # Using a small version of YOLOv5 for fast performance

# Load the guest database
guest_database = {
    "guest1": {"name": "Alice", "face_encoding": None, "drink": "Coffee"},
    "guest2": {"name": "Bob", "face_encoding": None, "drink": "Tea"},
    # Add more guests
}

# Load guest face encodings
for guest in guest_database.values():
    image = face_recognition.load_image_file(f"{guest['name']}.jpg")
    guest['face_encoding'] = face_recognition.face_encodings(image)[0]

# Initialize CV Bridge
bridge = CvBridge()

# Initialize SoundClient for text-to-speech
soundhandle = SoundClient()

# Capture image from Kinect
def image_callback(msg):
    # Convert ROS Image to OpenCV format
    frame = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
    
    # Detect faces using YOLOv5
    results = model(frame)
    
    # Process detected faces
    for result in results.xyxy[0]:
        x1, y1, x2, y2, conf, cls = result
        face_image = frame[int(y1):int(y2), int(x1):int(x2)]
        face_encoding = face_recognition.face_encodings(face_image)
        
        if face_encoding:
            face_encoding = face_encoding[0]
            # Compare face with the database
            matches = face_recognition.compare_faces(
                [guest['face_encoding'] for guest in guest_database.values()],
                face_encoding
            )
            name = "Unknown"
            drink = "Unknown"
            
            if True in matches:
                match_index = matches.index(True)
                guest_info = list(guest_database.values())[match_index]
                name = guest_info['name']
                drink = guest_info['drink']
            
            # Announce the guest's name and preferred drink
            soundhandle.say(f"Hello {name}, would you like some {drink}?")
        else:
            soundhandle.say("Unknown guest, please introduce yourself")
    
# Subscribe to the Kinect camera topic
image_sub = rospy.Subscriber('/camera/rgb/image_color', Image, image_callback)

# Keep the node running
rospy.spin()

