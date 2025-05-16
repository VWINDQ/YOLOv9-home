#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from gtts import gTTS
import os
import threading
import cv2
import numpy as np
import speech_recognition as sr

# Function to announce the current state
def announce_state(state_name):
    tts = gTTS(text=f"Entering {state_name} state.", lang='en')
    tts.save("/tmp/state_announcement.mp3")
    os.system("mpg321 /tmp/state_announcement.mp3")

def speak(text):
    """Convert text to speech and play it."""
    tts = gTTS(text=text, lang='en')
    tts.save("/tmp/response.mp3")
    os.system("mpg321 /tmp/response.mp3")

def recognize_speech_from_mic():
    """Recognize speech using the microphone."""
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    with microphone as source:
        print("Listening...")
        recognizer.adjust_for_ambient_noise(source)
        audio = recognizer.listen(source)

    try:
        # Recognize speech using Google Web Speech API
        response = recognizer.recognize_google(audio)
        print(f"You said: {response}")
        return response.lower()  # Convert to lower case for easier matching
    except sr.RequestError:
        print("API unavailable or unresponsive")
        return None
    except sr.UnknownValueError:
        print("Sorry, I didn't catch that.")
        return None

class SearchForPeople(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['person_found', 'searching', 'completed'], input_keys=['waypoints_in'], output_keys=['waypoints_out'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        self.bridge = CvBridge()
        self.current_waypoint = 0

    def execute(self, userdata):
        announce_state("Search For People")
        rospy.loginfo("[FSM] State: SEARCHING FOR PEOPLE")
        waypoints = userdata.waypoints_in

        if self.current_waypoint >= len(waypoints):
            return 'completed'

        # Move to the next waypoint
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        waypoint = waypoints[self.current_waypoint]
        goal.target_pose.pose.position.x = waypoint[0]
        goal.target_pose.pose.position.y = waypoint[1]
        goal.target_pose.pose.orientation.z = waypoint[2]
        goal.target_pose.pose.orientation.w = waypoint[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()

        # Simulate detecting a person at the waypoint
        rospy.sleep(5)  # Replace with actual detection logic
        person_detected = True  # Set to False if no person is detected

        if person_detected:
            return 'person_found'
        else:
            self.current_waypoint += 1
            userdata.waypoints_out = waypoints
            return 'searching'

class DetectGenderAndColor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'], output_keys=['gender_out', 'color_out'])
        self.bridge = CvBridge()
        self.gender_net = cv2.dnn.readNetFromCaffe('/home/user/age-and-gender-classification/model/deploy_gender2.prototxt', '/home/user/age-and-gender-classification/model/gender_net.caffemodel')
        self.gender_list = ['Male', 'Female']
        self.face_detected = False
        self.gender = None
        self.color = None
        self.image_received = False
        self.frame_count = 0  # Initialize a frame counter
        self.frame_skip = 10  # Process every 5th frame

    def detect_gender(self, face_img):
        blob = cv2.dnn.blobFromImage(face_img, 1.0, (227, 227), (78.4263377603, 87.7689143744, 114.895847746), swapRB=False)
        self.gender_net.setInput(blob)
        gender_preds = self.gender_net.forward()
        gender = self.gender_list[gender_preds[0].argmax()]
        return gender

    def detect_shirt_color(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_ranges = {
            "Red": ((0, 50, 50), (10, 255, 255)),
            "Green": ((35, 50, 50), (85, 255, 255)),
            "Blue": ((100, 50, 50), (130, 255, 255)),
            "Yellow": ((20, 50, 50), (30, 255, 255)),
            "White": ((0, 0, 200), (180, 20, 255)),
            "Purple": ((130, 50, 50), (160, 255, 255)),
            "Navy": ((100, 50, 30), (130, 255, 100)),
            "Black": ((0, 0, 0), (180, 255, 50))
        }
        color_detected = "Unknown"
        max_area = 0
        for color, (lower, upper) in color_ranges.items():
            mask = cv2.inRange(hsv_image, np.array(lower), np.array(upper))
            area = np.sum(mask > 0)
            if area > max_area:
                max_area = area
                color_detected = color
        return color_detected

    def image_callback(self, msg):
        self.frame_count += 1
        if self.frame_count % self.frame_skip != 0:
            return  # Skip this frame

        rospy.loginfo("Processing frame")
        self.image_received = True
        try:
            # Same processing as before
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            frame = cv2.resize(frame, (320, 240))  # Resize the image
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.2, minNeighbors=5, minSize=(30, 30))

            if len(faces) == 0:
                self.face_detected = False
                rospy.loginfo("No faces detected")
                return

            rospy.loginfo(f"Detected {len(faces)} face(s)")
            for (x, y, w, h) in faces:
                face_img = frame[y:y+h, x:x+w].copy()
                self.gender = self.detect_gender(face_img)
                shirt_area = frame[y + h: y + h + 50, x:x+w].copy()
                self.color = self.detect_shirt_color(shirt_area)

                self.face_detected = True
                return
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))
            return

    def execute(self, userdata):
        announce_state("Detect Gender And Color")
        rospy.loginfo("[FSM] State: DETECTING GENDER AND COLOR")
        camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)

        rospy.sleep(5)  # Wait for images to be processed

        if self.face_detected:
            userdata.gender_out = self.gender
            userdata.color_out = self.color
            return 'detected'
        return 'not_detected'

import subprocess

class ConfirmPersonName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['confirmed', 'not_confirmed'], input_keys=['name_in'], output_keys=['confirmed_name'])
    
    def check_lidar(self):
        """Check if Lidar is available and functioning."""
        try:
            # Check if Lidar topic is active
            output = subprocess.check_output(["rostopic", "list"])
            if "/scan" in output.decode():
                rospy.loginfo("Lidar is functioning correctly.")
                return True
            else:
                rospy.logwarn("Lidar is not detected. Attempting to restart...")
                return self.restart_lidar()
        except Exception as e:
            rospy.logerr(f"Error checking Lidar: {e}")
            return False

    def restart_lidar(self):
        """Attempt to restart the Lidar node."""
        try:
            subprocess.call(["rosnode", "kill", "/ydlidar_ros_driver_node"])
            rospy.sleep(2)  # Wait for node to fully shut down
            subprocess.call(["roslaunch", "ydlidar_ros_driver", "X2.launch"])
            rospy.sleep(5)  # Wait for Lidar to fully start up
            return self.check_lidar()  # Recheck if Lidar is now available
        except Exception as e:
            rospy.logerr(f"Failed to restart Lidar: {e}")
            return False

    def execute(self, userdata):
        announce_state("Confirm Person Name")
        rospy.loginfo("[FSM] State: CONFIRMING PERSON'S NAME")

        # Ask if the person is the expected name
        speak(f"Are you {userdata.name_in}? Please say yes or no.")

        # Listen for the response
        response = recognize_speech_from_mic()
        if response and ("yes" in response):
            speak("Thank you for confirming!")
            userdata.confirmed_name = userdata.name_in

            # Check if Lidar is still functioning before proceeding
            if not self.check_lidar():
                rospy.logerr("Lidar failed after confirmation. Aborting...")
                return 'not_confirmed'

            return 'confirmed'
        else:
            speak("Sorry, I will try again.")
            return 'not_confirmed'


class MoveToOwner(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_owner'], input_keys=['confirmed_name', 'gender_in', 'color_in', 'owner_position_in'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        announce_state("Move To Owner")
        rospy.loginfo("[FSM] State: MOVING TO OWNER")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Set the goal to the owner's location
        owner_position = userdata.owner_position_in
        goal.target_pose.pose.position.x = owner_position[0]
        goal.target_pose.pose.position.y = owner_position[1]
        goal.target_pose.pose.orientation.z = owner_position[2]
        goal.target_pose.pose.orientation.w = owner_position[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()

        return 'at_owner'

class AnnounceToOwner(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['announcement_done'], input_keys=['confirmed_name', 'gender_in', 'color_in'])

    def execute(self, userdata):
        announce_state("Announce To Owner")
        rospy.loginfo("[FSM] State: ANNOUNCING TO OWNER")
        text = f"{userdata.confirmed_name} is {userdata.gender_in} and is wearing a {userdata.color_in} shirt."
        speak(text)
        return 'announcement_done'

def main():
    rospy.init_node('fsm_bgn')

    sm = smach.StateMachine(outcomes=['completed'])
    sm.userdata.name = "John"  # Replace with the actual name to confirm
    sm.userdata.waypoints = [
    	 (2.55, 2.57, 0.00247, 1.0),  # Waypoint 1
      #  (2.55, 2.57, 0.00247, 1.0),  # Waypoint 2
      #  (2.55, 2.57, 0.00247, 1.0),    # Waypoint 3
    ]
    sm.userdata.owner_position = (-0.00367, -0.00969, 0.0859, 1.0)  # Replace with the actual owner's position on the map

    with sm:
        smach.StateMachine.add('SEARCH', SearchForPeople(), transitions={'person_found':'DETECT', 'searching':'SEARCH', 'completed':'completed'},
                               remapping={'waypoints_in':'waypoints', 'waypoints_out':'waypoints'})
        smach.StateMachine.add('DETECT', DetectGenderAndColor(),
                               transitions={'detected':'CONFIRM_NAME', 'not_detected':'SEARCH'},
                               remapping={'gender_out':'gender', 'color_out':'color'})
        smach.StateMachine.add('CONFIRM_NAME', ConfirmPersonName(),
                               transitions={'confirmed':'MOVE_TO_OWNER', 'not_confirmed':'SEARCH'},
                               remapping={'name_in':'name', 'confirmed_name':'confirmed_name'})
        smach.StateMachine.add('MOVE_TO_OWNER', MoveToOwner(), transitions={'at_owner':'ANNOUNCE'},
                               remapping={'confirmed_name':'confirmed_name', 'gender_in':'gender', 'color_in':'color', 'owner_position_in':'owner_position'})
        smach.StateMachine.add('ANNOUNCE', AnnounceToOwner(),
                               transitions={'announcement_done':'completed'},
                               remapping={'confirmed_name':'confirmed_name', 'gender_in':'gender', 'color_in':'color'})

    outcome = sm.execute()
    rospy.loginfo("[FSM] Final Outcome: %s", outcome)

if __name__ == '__main__':
    main()

