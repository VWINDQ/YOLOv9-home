#!/usr/bin/env python3

import rospy
import smach
import smach_ros
import torch
from gtts import gTTS
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
import random
import os
import speech_recognition as sr

# YOLOv5 model for seat detection (if needed for seat detection)
seat_model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/user/yolov5/runs/train/seat_detection2/weights/best.pt')

# Initialize guest name
guest_name = None

# Store positions for robot navigation (predefined positions)
START_POINT = {'x': 5.04, 'y': -0.281, 'w': 1.0}  # Facing forward
SEAT_LOCATION = {'x': 3.94, 'y': -0.183, 'w': 0.00247}  # No turning when reaching the seat

# TTS function to make the robot speak
def speak(text):
    tts = gTTS(text=text, lang='en')
    tts.save("/tmp/voice.mp3")
    os.system("mpg321 /tmp/voice.mp3")

# Function to move robot to a specific location
def move_to_goal(x, y, orientation_w=1.0):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    
    # Set the position for the robot to move to
    goal.target_pose.pose.position.x = x
    goal.target_pose.pose.position.y = y
    
    # Set the orientation (initially, facing forward)
    goal.target_pose.pose.orientation.w = orientation_w  # No turning at seat
    
    client.send_goal(goal)
    client.wait_for_result()

# Function to turn the robot 90 degrees to the left
def turn_90_degrees_left():
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()

    # Keep the same position but update the orientation for a 90-degree turn to the left
    goal.target_pose.pose.position.x = SEAT_LOCATION['x']
    goal.target_pose.pose.position.y = SEAT_LOCATION['y']
    
    # Set the orientation for a 90-degree left turn (z = 0.7071, w = 0.7071)
    goal.target_pose.pose.orientation.z = 0.7071
    goal.target_pose.pose.orientation.w = 0.7071  # Rotate 90 degrees to the left
    
    client.send_goal(goal)
    client.wait_for_result()

# Function to listen for the guest's name using speech recognition
def listen_for_name():
    recognizer = sr.Recognizer()
    microphone = sr.Microphone()

    speak("Please tell me your name.")
    
    with microphone as source:
        recognizer.adjust_for_ambient_noise(source)  # Adjust for background noise
        rospy.loginfo("Listening for guest's name...")
        audio = recognizer.listen(source)
    
    try:
        guest_name = recognizer.recognize_google(audio)
        rospy.loginfo(f"Guest's name recognized: {guest_name}")
        speak(f"Nice to meet you, {guest_name}. Please follow me to your seat.")
        return guest_name
    except sr.UnknownValueError:
        rospy.logwarn("Could not understand the audio.")
        speak("I'm sorry, I didn't catch that. Could you please repeat your name?")
        return listen_for_name()  # Ask again if the name was not recognized
    except sr.RequestError:
        rospy.logerr("Speech recognition service is unavailable.")
        speak("I'm having trouble understanding. Please try again.")
        return None

# Define state WAIT_FOR_NAME
class WaitForName(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['name_heard'])
    
    def execute(self, userdata):
        global guest_name
        guest_name = listen_for_name()  # Wait for the guest to say their name
        if guest_name:
            return 'name_heard'

# Define state GUIDE_TO_SEAT
class GuideToSeat(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['seated'])
    
    def execute(self, userdata):
        rospy.loginfo(f"Guiding {guest_name} to seat...")
        move_to_goal(SEAT_LOCATION['x'], SEAT_LOCATION['y'], SEAT_LOCATION['w'])  # Move to seat without turning
        return 'seated'

# Define state TURN_90_LEFT
class Turn90Left(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['turned'])
    
    def execute(self, userdata):
        rospy.loginfo("Turning 90 degrees to the left at seat location...")
        turn_90_degrees_left()  # Rotate robot 90 degrees to the left
        return 'turned'

# Define state INTRODUCE_GUEST
class IntroduceGuest(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['introduced'])
    
    def execute(self, userdata):
        speak(f"Everyone, this is {guest_name}.")
        return 'introduced'

# Main function
def main():
    rospy.init_node('guest_welcome_fsm')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['end'])

    # Add states to the state machine
    with sm:
        smach.StateMachine.add('WAIT_FOR_NAME', WaitForName(), 
                               transitions={'name_heard':'GUIDE_TO_SEAT'})

        smach.StateMachine.add('GUIDE_TO_SEAT', GuideToSeat(),
                               transitions={'seated':'TURN_90_LEFT'})

        smach.StateMachine.add('TURN_90_LEFT', Turn90Left(),
                               transitions={'turned':'INTRODUCE_GUEST'})

        smach.StateMachine.add('INTRODUCE_GUEST', IntroduceGuest(),
                               transitions={'introduced':'end'})

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()

