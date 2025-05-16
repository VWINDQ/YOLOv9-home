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
import cv2
import numpy as np
from geometry_msgs.msg import Twist, Pose
import speech_recognition as sr
import math

def announce_state(state_name):
    tts = gTTS(text=f"Entering {state_name} state.", lang='en')
    tts.save("/tmp/state_announcement.mp3")
    os.system("mpg321 /tmp/state_announcement.mp3")

def speak(text):
    tts = gTTS(text=text, lang='en')
    tts.save("/tmp/response.mp3")
    os.system("mpg321 /tmp/response.mp3")

def recognize_speech_from_mic(timeout=5):
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    with microphone as source:
        print("Listening...")
        recognizer.adjust_for_ambient_noise(source)
        try:
            audio = recognizer.listen(source, timeout=timeout)
            response = recognizer.recognize_google(audio)
            print(f"You said: {response}")
            return response.lower()
        except sr.WaitTimeoutError:
            print("No response detected within the timeout.")
            return None
        except sr.RequestError:
            print("API unavailable or unresponsive")
            return None
        except sr.UnknownValueError:
            print("Sorry, I didn't catch that.")
            return None

class MoveToDesignatedSpot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_spot', 'failed'], input_keys=['designated_position_in'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        announce_state("Move To Designated Spot")
        rospy.loginfo("[FSM] State: MOVING TO DESIGNATED SPOT")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        designated_position = userdata.designated_position_in
        goal.target_pose.pose.position.x = designated_position[0]
        goal.target_pose.pose.position.y = designated_position[1]
        goal.target_pose.pose.orientation.z = designated_position[2]
        goal.target_pose.pose.orientation.w = designated_position[3]

        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            return 'at_spot'
        else:
            return 'failed'

class ScanForFaces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['face_detected', 'face_not_detected'], output_keys=['current_face_position_out'])
        self.bridge = CvBridge()
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.face_detected = False
        self.face_position = None
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)

    def camera_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            faces = face_cascade.detectMultiScale(gray, 1.3, 5)

            if len(faces) > 0:
                # เลือกใบหน้าที่ใหญ่ที่สุด (ใกล้ที่สุด)
                largest_face = max(faces, key=lambda rect: rect[2] * rect[3])
                (x, y, w, h) = largest_face
                self.face_position = (x + w // 2, y + h // 2)
                self.face_detected = True
                rospy.loginfo("Face detected.")
            else:
                self.face_detected = False
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def execute(self, userdata):
        announce_state("Scan For Faces")
        rospy.loginfo("[FSM] State: SCANNING FOR FACES")

        rate = rospy.Rate(10)
        twist = Twist()
        twist.angular.z = 0.2  # ลดความเร็วในการหมุน

        while not rospy.is_shutdown():
            if self.face_detected:
                x, _ = self.face_position
                if abs(x - 320) < 20:  # ขยายกรอบสำหรับการหยุดหมุน
                    self.twist_pub.publish(Twist())  # หยุดหมุนเมื่อใบหน้าอยู่ใกล้ศูนย์กลาง
                    userdata.current_face_position_out = self.face_position
                    return 'face_detected'
                elif x < 320:
                    twist.angular.z = 0.1  # หมุนซ้ายอย่างช้าๆ
                else:
                    twist.angular.z = -0.1  # หมุนขวาอย่างช้าๆ
                self.twist_pub.publish(twist)
            else:
                self.twist_pub.publish(twist)  # หมุนต่อไปเพื่อค้นหาใบหน้า
            rate.sleep()

        return 'face_not_detected'
        
class DetectGenderAndColor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'], output_keys=['gender_out', 'color_out'])
        self.bridge = CvBridge()
        self.gender_net = cv2.dnn.readNetFromCaffe('/home/user/age-and-gender-classification/model/deploy_gender2.prototxt', '/home/user/age-and-gender-classification/model/gender_net.caffemodel')
        self.gender_list = ['Male', 'Female']
        self.face_detected = False
        self.gender = None
        self.color = None

    def detect_gender(self, face_img):
        blob = cv2.dnn.blobFromImage(face_img, 1.0, (227, 227), (78.4263377603, 87.7689143744, 114.895847746), swapRB=False)
        self.gender_net.setInput(blob)
        gender_preds = self.gender_net.forward()
        gender = self.gender_list[gender_preds[0].argmax()]
        return gender

    def detect_shirt_color(self, image):
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        color_ranges = {
            "White": ((0, 50, 50), (10, 255, 255)),
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
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
            faces = face_cascade.detectMultiScale(gray, scaleFactor=1.1, minNeighbors=5, minSize=(30, 30))

            if len(faces) == 0:
                self.face_detected = False
                return

            for (x, y, w, h) in faces:
                face_img = frame[y:y+h, x:x+w].copy()
                self.gender = self.detect_gender(face_img)

                shirt_area = frame[y + h: y + h + 50, x:x+w].copy()
                self.color = self.detect_shirt_color(shirt_area)

                self.face_detected = True
                return
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def execute(self, userdata):
        announce_state("Detect Gender And Color")
        rospy.loginfo("[FSM] State: DETECTING GENDER AND COLOR")
        camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)

        rospy.sleep(5)  # รอให้มีการประมวลผลภาพ

        if self.face_detected:
            userdata.gender_out = self.gender
            userdata.color_out = self.color
            return 'detected'
        return 'not_detected'

class ConfirmPersonName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['confirmed', 'not_confirmed', 'try_next'], input_keys=['guests_in', 'current_guest_index_in'], output_keys=['confirmed_name', 'current_guest_index_out'])
        self.attempts = 0

    def execute(self, userdata):
        announce_state("Confirm Person Name")
        rospy.loginfo("[FSM] State: CONFIRMING PERSON'S NAME")

        guests = userdata.guests_in
        current_guest_index = userdata.current_guest_index_in

        if current_guest_index >= len(guests):
            rospy.logwarn("All guests have been asked. Returning to the owner.")
            return 'try_next'

        current_guest_name = guests[current_guest_index]

        if self.attempts == 0:
            speak(f"Are you {current_guest_name}? Please say yes or no.")
        else:
            speak("Sorry, I didn't catch that. Can you please tell me your name again?")

        response = recognize_speech_from_mic()

        if response and "yes" in response and self.attempts == 0:
            speak("Thank you for confirming!")
            userdata.confirmed_name = current_guest_name
            return 'confirmed'
        elif response and "no" in response:
            userdata.current_guest_index_out = current_guest_index + 1
            return 'try_next'
        elif response is None:
            userdata.current_guest_index_out = current_guest_index + 1
            return 'try_next'
        else:
            self.attempts += 1
            return 'not_confirmed'

class ReturnToOwner(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_owner'], input_keys=['owner_position_in'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        announce_state("Return to Owner")
        rospy.loginfo("[FSM] State: RETURNING TO OWNER")
        
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

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
        announce_state("Announce to Owner")
        rospy.loginfo("[FSM] State: ANNOUNCING TO OWNER")
        text = f"{userdata.confirmed_name} is {userdata.gender_in} and is wearing a {userdata.color_in} shirt."
        speak(text)
        return 'announcement_done'

class CheckIterationCount(smach.State):
    def __init__(self, max_iterations=3):
        smach.State.__init__(self, outcomes=['continue', 'reset', 'completed'], input_keys=['iteration_count_in'], output_keys=['iteration_count_out'])
        self.max_iterations = max_iterations

    def execute(self, userdata):
        if userdata.iteration_count_in >= self.max_iterations:
            userdata.iteration_count_out = 0  # Reset count
            return 'reset'  # Transition to reset state (SCAN_FOR_FACES)
        else:
            userdata.iteration_count_out = userdata.iteration_count_in + 1
            return 'continue'

def main():
    rospy.init_node('fsm_robot')

    sm = smach.StateMachine(outcomes=['completed'])
    sm.userdata.guests = ["John", "Alice", "Bob"]  # List of guest names
    sm.userdata.current_guest_index = 0  # Start with the first guest
    sm.userdata.designated_position = (0.474, -3.07, -0.00143, 1.0) # Replace with actual designated spot coordinates
    sm.userdata.owner_position = (4, -3.97, 0.00638, 1.0)  # Replace with the actual owner's position on the map
    sm.userdata.iteration_count = 2  # Initialize the iteration count
    sm.userdata.visited_locations = []  # Store locations of visited people

    with sm:
        smach.StateMachine.add('MOVE_TO_SPOT', MoveToDesignatedSpot(),
                               transitions={'at_spot':'SCAN_FOR_FACES', 'failed':'completed'},
                               remapping={'designated_position_in':'designated_position'})

        smach.StateMachine.add('SCAN_FOR_FACES', ScanForFaces(),
                               transitions={'face_detected':'DETECT', 'face_not_detected':'SCAN_FOR_FACES'},
                               remapping={'current_face_position_out':'current_face_position'})

        smach.StateMachine.add('DETECT', DetectGenderAndColor(),
                               transitions={'detected':'CONFIRM_NAME', 'not_detected':'SCAN_FOR_FACES'},
                               remapping={'gender_out':'gender', 'color_out':'color'})

        smach.StateMachine.add('CONFIRM_NAME', ConfirmPersonName(),
                               transitions={'confirmed':'RETURN_TO_OWNER', 'not_confirmed':'CONFIRM_NAME', 'try_next':'CHECK_ITERATION_COUNT'},
                               remapping={'guests_in':'guests', 'current_guest_index_in':'current_guest_index', 'confirmed_name':'confirmed_name', 'current_guest_index_out':'current_guest_index'})

        smach.StateMachine.add('RETURN_TO_OWNER', ReturnToOwner(),
                               transitions={'at_owner':'ANNOUNCE_TO_OWNER'},
                               remapping={'owner_position_in':'owner_position'})

        smach.StateMachine.add('ANNOUNCE_TO_OWNER', AnnounceToOwner(),
                               transitions={'announcement_done':'CHECK_ITERATION_COUNT'},
                               remapping={'confirmed_name':'confirmed_name', 'gender_in':'gender', 'color_in':'color'})

        smach.StateMachine.add('CHECK_ITERATION_COUNT', CheckIterationCount(max_iterations=3),
                       transitions={'continue':'CONFIRM_NAME', 
                                    'reset':'SCAN_FOR_FACES', 
                                    'completed':'completed'},
                       remapping={'iteration_count_in':'iteration_count', 'iteration_count_out':'iteration_count'})


    outcome = sm.execute()
    rospy.loginfo("[FSM] Final Outcome: %s", outcome)

if __name__ == '__main__':
    main()
