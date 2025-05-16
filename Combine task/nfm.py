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
import tf

# Helper function for speech
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

class MoveToWaypoint(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['at_waypoint', 'failed'], 
                             input_keys=['waypoints', 'current_guest_index', 'rooms'], 
                             output_keys=['current_room_out'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        announce_state("Move To Waypoint")
        rospy.loginfo("[FSM] State: MOVING TO WAYPOINT")

        # Get the current waypoint based on current_guest_index
        index = userdata.current_guest_index

        # Ensure that the waypoints are floats
        waypoint = userdata.waypoints[index]

        if len(waypoint) != 4:
            rospy.logerr("Waypoint data is not in the correct format")
            return 'failed'

        # Prepare the move goal
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Extract individual values from the waypoint tuple and ensure they are floats
        try:
            goal.target_pose.pose.position.x = float(waypoint[0])
            goal.target_pose.pose.position.y = float(waypoint[1])

            # Extract the orientation directly from waypoint (z and w values)
            goal.target_pose.pose.orientation.z = float(waypoint[2])
            goal.target_pose.pose.orientation.w = float(waypoint[3])

        except ValueError as e:
            rospy.logerr(f"Invalid waypoint values: {e}")
            return 'failed'

        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            # Set the current room for reporting
            userdata.current_room_out = userdata.rooms[index]
            return 'at_waypoint'
        else:
            return 'failed'



class ScanForFaces(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['face_detected', 'face_not_detected', 'too_many_rotations'], 
                             output_keys=['current_face_position_out'])
        self.bridge = CvBridge()
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.face_detected = False
        self.face_position = None
        self.image_sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.camera_callback)
        self.rotation_count = 0  # นับจำนวนรอบที่หุ่นหมุน

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

        self.rotation_count = 0  # เริ่มนับจำนวนรอบการหมุน

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
                self.rotation_count += 1

                # ถ้าหมุนเกินจำนวนรอบที่กำหนด
                if self.rotation_count >= 200:  # ตัวอย่าง: หมุน 2 รอบ (200 ticks)
                    return 'too_many_rotations'

            rate.sleep()

        return 'face_not_detected'



class DetectGenderAndColor(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['detected', 'not_detected'], output_keys=['gender_out', 'color_out', 'current_face_position_out'])
        self.bridge = CvBridge()
        self.face_position = None
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
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
            "White": ((0, 50, 50), (10, 150, 150)),           
            "Green": ((35, 50, 50), (85, 255, 255)),
            "Light Green": ((35, 50, 50), (65, 255, 255)),
            "Dark Green": ((65, 100, 100), (85, 255, 255)),
            "Blue": ((100, 50, 50), (130, 255, 255)),
            "Light Blue": ((100, 50, 50), (120, 200, 255)),
            "Dark Blue": ((120, 100, 100), (130, 255, 255)),
            "Yellow": ((20, 50, 50), (30, 255, 255)),
            "White": ((0, 0, 200), (180, 20, 255)),
            "Purple": ((130, 50, 50), (160, 255, 255)),
            "Navy": ((100, 50, 30), (130, 255, 100)),
            "Black": ((0, 0, 0), (180, 255, 50)),
            "Orange": ((5, 100, 100), (15, 255, 255)),
            "Pink": ((150, 50, 50), (170, 255, 255))
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
                self.face_position = (x + w // 2, y + h // 2)  # Store face position
                self.face_detected = True
                return
        except CvBridgeError as e:
            rospy.logerr("CvBridge Error: {0}".format(e))

    def execute(self, userdata):
        announce_state("Detect Gender And Color")
        rospy.loginfo("[FSM] State: DETECTING GENDER AND COLOR")
        camera_topic = rospy.get_param('~camera_topic', '/camera/rgb/image_raw')
        image_sub = rospy.Subscriber(camera_topic, Image, self.image_callback)

        rospy.sleep(5)  # Wait for image processing

        if self.face_detected:
            userdata.gender_out = self.gender
            userdata.color_out = self.color
            userdata.current_face_position_out = self.face_position

            # Move to the detected face position (approach the person)
            goal = MoveBaseGoal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.header.stamp = rospy.Time.now()
            
            # Assuming the robot should move to the face position (detected position)
            # You may need to adjust the coordinates to be in the map frame.
            goal.target_pose.pose.position.x = 0.5  # Set an appropriate value for how close you want the robot to get
            goal.target_pose.pose.position.y = 0.0
            goal.target_pose.pose.orientation.w = 1.0  # Face the person

            self.client.send_goal(goal)
            self.client.wait_for_result()

            return 'detected'
        return 'not_detected'


class ConfirmPersonName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['confirmed', 'skip'], 
                             input_keys=['available_names_in', 'current_face_position_in', 'visited_locations_in'], 
                             output_keys=['confirmed_name', 'available_names_out', 'visited_locations_out'])
        self.attempts = 0

    def is_location_visited(self, current_position, visited_locations):
        """Check if the current position has already been visited."""
        distance_threshold = 1.0
        for location in visited_locations:
            distance = math.sqrt((current_position[0] - location[0]) ** 2 + (current_position[1] - location[1]) ** 2)
            if distance < distance_threshold:
                return True
        return False

    def execute(self, userdata):
        announce_state("Confirm Person Name")
        rospy.loginfo("[FSM] State: CONFIRMING PERSON'S NAME")

        available_names = userdata.available_names_in
        current_position = userdata.current_face_position_in
        visited_locations = userdata.visited_locations_in

        # Check if this location has already been visited
        if self.is_location_visited(current_position, visited_locations):
            rospy.loginfo("This location has already been visited. Skipping...")
            return 'skip'

        # Ask for the user's name
        speak("What is your name? Please tell me your name.")
        response = recognize_speech_from_mic()

        if response:
            # Check if the name is in the list
            if response.capitalize() in available_names:
                userdata.confirmed_name = response.capitalize()
                speak(f"Thank you! Confirmed that you are {response.capitalize()}")

                # Record the visited location
                visited_locations.append(current_position)
                userdata.visited_locations_out = visited_locations

                # Remove the confirmed name from the list of available names
                available_names.remove(response.capitalize())
                userdata.available_names_out = available_names

                return 'confirmed'
            else:
                # If the name is not in the list
                speak(f"Sorry, the name {response.capitalize()} is not in the expected list.")
                return 'skip'
        else:
            # If no response or unclear response
            speak("I didn't hear you. Please try again.")
            return 'skip'




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
    def __init__(self, max_iterations=4):
        smach.State.__init__(self, outcomes=['continue', 'completed'], 
                             input_keys=['iteration_count_in', 'current_guest_index'],
                             output_keys=['iteration_count_out', 'current_guest_index_out'])
        self.max_iterations = max_iterations

    def execute(self, userdata):
        if userdata.iteration_count_in >= self.max_iterations:
            userdata.iteration_count_out = 0  # Reset count
            userdata.current_guest_index_out = 0  # Reset guest index
            return 'completed'  # End the task after reaching the maximum iterations
        else:
            userdata.iteration_count_out = userdata.iteration_count_in + 1
            # Increment the guest index to move to the next waypoint
            userdata.current_guest_index_out = (userdata.current_guest_index + 1) % len(userdata.waypoints)
            return 'continue'


def main():
    rospy.init_node('fsm_robot')

    sm = smach.StateMachine(outcomes=['completed'])
    
    # Updated list of available guest names
    sm.userdata.available_names = [
        "Julia", "Emma", "Sara", "Laura", "Susan", "John", 
        "Lucas", "William", "Kevin", "Peter", "Robin", 
        "Jeffery", "Tan", "Sarah", "Adam"
    ]
    
    sm.userdata.current_guest_index = 0  # Start with the first guest
    sm.userdata.waypoints = [
     (4.63, 1.03, 0.0, 1.0),	#"Living Room"
     (0.221, -2.5, -0.00534, 1.0), #"Bedroom"
     (-0.00489, 0.29, 0.00247, 1.0), #"Kitchen"
     (3.52, -2.6, 0.0, 1.0), #"Study"
    ]
    sm.userdata.rooms = ["Living Room", "Kitchen", "Bedroom", "Study"]  # Room names corresponding to waypoints
    sm.userdata.owner_position = (4, -3.97, 0.00638, 1.0)  # Owner's location
    sm.userdata.iteration_count = 3  # Initialize the iteration count
    sm.userdata.visited_locations = []  # Store locations of visited people
    sm.userdata.current_face_position = (0, 0)  # Initialize the current face position

    with sm:
        smach.StateMachine.add('MOVE_TO_SPOT', MoveToWaypoint(),
                               transitions={'at_waypoint': 'SCAN_FOR_FACES', 'failed': 'completed'},
                               remapping={'waypoint_in': 'waypoints', 'room_in': 'rooms', 'current_room_out': 'current_room'})

        smach.StateMachine.add('SCAN_FOR_FACES', ScanForFaces(),
                               transitions={'face_detected': 'DETECT', 'face_not_detected': 'SCAN_FOR_FACES', 'too_many_rotations': 'MOVE_TO_SPOT'},
                               remapping={'current_face_position_out': 'current_face_position'})

        smach.StateMachine.add('DETECT', DetectGenderAndColor(),
                               transitions={'detected': 'CONFIRM_NAME', 'not_detected': 'SCAN_FOR_FACES'},
                               remapping={'gender_out': 'gender', 'color_out': 'color'})

        smach.StateMachine.add('CONFIRM_NAME', ConfirmPersonName(),
                               transitions={'confirmed': 'RETURN_TO_OWNER', 'skip': 'SCAN_FOR_FACES'},
                               remapping={'available_names_in': 'available_names', 'confirmed_name': 'confirmed_name', 'available_names_out': 'available_names', 'visited_locations_in': 'visited_locations', 'visited_locations_out': 'visited_locations', 'current_face_position_in': 'current_face_position'})

        smach.StateMachine.add('RETURN_TO_OWNER', ReturnToOwner(),
                               transitions={'at_owner': 'ANNOUNCE_TO_OWNER'},
                               remapping={'owner_position_in': 'owner_position'})

        smach.StateMachine.add('ANNOUNCE_TO_OWNER', AnnounceToOwner(),
                               transitions={'announcement_done': 'CHECK_ITERATION_COUNT'},
                               remapping={'confirmed_name': 'confirmed_name', 'gender_in': 'gender', 'color_in': 'color', 'current_room_in': 'current_room'})

        smach.StateMachine.add('CHECK_ITERATION_COUNT', CheckIterationCount(max_iterations=4),
                               transitions={'continue': 'MOVE_TO_SPOT', 'completed': 'completed'},
                               remapping={'iteration_count_in': 'iteration_count', 'iteration_count_out': 'iteration_count'})

    outcome = sm.execute()
    rospy.loginfo("[FSM] Final Outcome: %s", outcome)

if __name__ == '__main__':
    main()


