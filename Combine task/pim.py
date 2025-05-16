#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import smach
import smach_ros
from gtts import gTTS
import os
import time
import speech_recognition as sr
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import actionlib
from std_msgs.msg import String

class RobotBehavior:
    def __init__(self):
        # เริ่มต้น Node
        rospy.init_node('welcome_robot', anonymous=True)
        # Client สำหรับ move_base
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        # Publisher สำหรับส่งคำสั่งไปยังแขนกล
        self.arm_pub = rospy.Publisher('Test', String, queue_size=10)
        # กำหนด Waypoints
        self.kitchen_waypoint = {'x': 5.55, 'y': 1.79, 'z': 0.00247, 'w': 1.0}
        self.front_door_waypoint = {'x': -0.0156, 'y': -0.0206, 'z': 0.086, 'w': 1.0}
        self.living_room_waypoint = {'x': 1.99, 'y': 2.81, 'z': 0.00247, 'w': 1.0}

    def speak(self, text):
        tts = gTTS(text=text, lang='th')
        tts.save('/tmp/temp.mp3')
        os.system('mpg321 /tmp/temp.mp3')

    def wait_for_response(self):
        recognizer = sr.Recognizer()
        with sr.Microphone() as source:
            print("กำลังฟัง...")
            audio = recognizer.listen(source)
        try:
            response = recognizer.recognize_google(audio, language="th-TH")
            print(f"Response: {response}")
            if "ใช่" in response:
                return "yes"
            elif "ไม่" in response:
                return "no"
            else:
                self.speak("ไม่เข้าใจ กรุณาลองอีกครั้ง")
                return self.wait_for_response()
        except sr.UnknownValueError:
            self.speak("ขอโทษค่ะ ฟังไม่ชัดเจน กรุณาลองอีกครั้ง")
            return self.wait_for_response()

    def move_to_waypoint(self, waypoint):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint['x']
        goal.target_pose.pose.position.y = waypoint['y']
        goal.target_pose.pose.orientation.z = waypoint['z']
        goal.target_pose.pose.orientation.w = waypoint['w']
        rospy.loginfo(f"Moving to waypoint: {waypoint}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def rotate_at_waypoint(self, waypoint, z, w):
        # หมุนหุ่นยนต์โดยเปลี่ยน orientation ที่ตำแหน่งเดิม
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = waypoint['x']
        goal.target_pose.pose.position.y = waypoint['y']
        goal.target_pose.pose.orientation.z = z
        goal.target_pose.pose.orientation.w = w
        rospy.loginfo(f"Rotating at waypoint: {waypoint} to orientation z:{z}, w:{w}")
        self.client.send_goal(goal)
        self.client.wait_for_result()

    def control_arm(self, command):
        msg = String()
        msg.data = command
        self.arm_pub.publish(msg)
        rospy.loginfo(f"ส่งคำสั่ง '{command}' ไปยังแขนกล")
        time.sleep(1)  # รอให้แขนกลดำเนินการ

# กำหนดสถานะต่างๆ โดยใช้ SMACH

class Idle(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['go_to_door'])
        self.robot = robot

    def execute(self, userdata):
        rospy.loginfo("หุ่นยนต์อยู่ที่ห้องครัวกับเจ้าบ้าน")
        rospy.sleep(2)  # รอ 2 วินาที
        rospy.loginfo("แขกกดกริ่ง")
        return 'go_to_door'

class GoToDoor(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['arrived_door'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.move_to_waypoint(self.robot.front_door_waypoint)
        return 'arrived_door'

class RotateAtDoor(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['greet_guest'])
        self.robot = robot

    def execute(self, userdata):
        # หมุน 180 องศา
        new_z = 1.0  # ค่า z สำหรับการหมุน 180 องศา
        new_w = 0.0  # ค่า w สำหรับการหมุน 180 องศา
        self.robot.rotate_at_waypoint(self.robot.front_door_waypoint, new_z, new_w)
        return 'greet_guest'

class GreetGuest(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['guest_confirmed', 'guest_denied'])
        self.robot = robot

    def execute(self, userdata):
        guest_name = "สมชาย"
        self.robot.speak(f"สวัสดีค่ะ คุณ{guest_name} ใช่หรือไม่")
        response = self.robot.wait_for_response()
        if response == "yes":
            return 'guest_confirmed'
        else:
            self.robot.speak("ขอโทษค่ะ อาจจะเป็นการเข้าใจผิด")
            return 'guest_denied'

class AskCarryBag(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['bag_yes', 'bag_no'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.speak("คุณต้องการให้ถือกระเป๋าให้ไหม")
        response = self.robot.wait_for_response()
        if response == "yes":
            return 'bag_yes'
        else:
            return 'bag_no'

class ReceiveBag(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['bag_received'])
        self.arduino_pub = rospy.Publisher('Test', String, queue_size=10)  # Assumed topic for the arm
        self.robot = robot

    def execute(self, userdata):
        self.robot.speak("กรุณายื่นกระเป๋าใส่ที่มือฉันหน่อยค่ะ")
        self.arduino_pub.publish("B")  # Initial pickup action
        rospy.sleep(5) 
        self.arduino_pub.publish("D")  # Arm move to pickup
        rospy.sleep(5) 
        self.arduino_pub.publish("C")  # Close gripper
        rospy.sleep(3) 
        self.arduino_pub.publish("E")  # Retract arm
        rospy.sleep(3)   # Wait for the arm to complete its movement
        return 'bag_received'

class LeadToLivingRoom(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['arrived_living_room'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.speak("ขอบคุณค่ะ กรุณาเดินตามฉันมา")
        self.robot.move_to_waypoint(self.robot.living_room_waypoint)
        return 'arrived_living_room'

class InformGuestWait(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['go_to_kitchen'])
        self.arduino_pub = rospy.Publisher('Test', String, queue_size=10)  # Assumed topic for the arm 
        self.robot = robot

    def execute(self, userdata):
        self.robot.speak("เชิญนั่งที่ว่างได้เลยค่ะ เดี๋ยวฉันไปตามเจ้าบ้านให้ กรุณารอสักครู่")
        self.arduino_pub.publish("B")  # Initial pickup action
        rospy.sleep(5) 
        self.arduino_pub.publish("D")  # Arm move to pickup
        rospy.sleep(5) 
        self.arduino_pub.publish("E")  # Retract arm
        rospy.sleep(3)   # Wait for the arm to complete its movement
        
        return 'go_to_kitchen'

class GoToKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['arrived_kitchen'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.move_to_waypoint(self.robot.kitchen_waypoint)
        return 'arrived_kitchen'

class InformOwner(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['deliver_drink'])
        self.robot = robot

    def execute(self, userdata):
        guest_name = "สมชาย"
        self.robot.speak(f"มีแขกชื่อ{guest_name} มารอแล้วค่ะ กรุณาวางน้ำที่ฉันด้วยค่ะ")
        rospy.sleep(5)  # รอให้เจ้าบ้านวางน้ำบนหุ่นยนต์
        return 'deliver_drink'

class DeliverDrink(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['drink_delivered'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.move_to_waypoint(self.robot.living_room_waypoint)
        self.robot.speak("เอาน้ำมาให้ค่ะ กรุณารอเจ้าบ้านสักครู่")
        return 'drink_delivered'

class WaitAfterDelivery(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['return_to_kitchen'])
        self.robot = robot

    def execute(self, userdata):
        rospy.sleep(5)
        return 'return_to_kitchen'

class ReturnToKitchen(smach.State):
    def __init__(self, robot):
        smach.State.__init__(self, outcomes=['end'])
        self.robot = robot

    def execute(self, userdata):
        self.robot.move_to_waypoint(self.robot.kitchen_waypoint)
        rospy.loginfo("หุ่นยนต์กลับไปยังห้องครัว")
        return 'end'

def main():
    robot = RobotBehavior()

    # สร้าง State Machine
    sm = smach.StateMachine(outcomes=['END'])

    with sm:
        smach.StateMachine.add('IDLE', Idle(robot), transitions={'go_to_door':'GOTO_DOOR'})
        smach.StateMachine.add('GOTO_DOOR', GoToDoor(robot), transitions={'arrived_door':'ROTATE_AT_DOOR'})
        smach.StateMachine.add('ROTATE_AT_DOOR', RotateAtDoor(robot), transitions={'greet_guest':'GREET_GUEST'})
        smach.StateMachine.add('GREET_GUEST', GreetGuest(robot), transitions={'guest_confirmed':'ASK_CARRY_BAG', 'guest_denied':'RETURN_TO_KITCHEN'})
        smach.StateMachine.add('ASK_CARRY_BAG', AskCarryBag(robot), transitions={'bag_yes':'RECEIVE_BAG', 'bag_no':'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add('RECEIVE_BAG', ReceiveBag(robot), transitions={'bag_received':'LEAD_TO_LIVING_ROOM'})
        smach.StateMachine.add('LEAD_TO_LIVING_ROOM', LeadToLivingRoom(robot), transitions={'arrived_living_room':'INFORM_GUEST_WAIT'})
        smach.StateMachine.add('INFORM_GUEST_WAIT', InformGuestWait(robot), transitions={'go_to_kitchen':'GO_TO_KITCHEN'})
        smach.StateMachine.add('GO_TO_KITCHEN', GoToKitchen(robot), transitions={'arrived_kitchen':'INFORM_OWNER'})
        smach.StateMachine.add('INFORM_OWNER', InformOwner(robot), transitions={'deliver_drink':'DELIVER_DRINK'})
        smach.StateMachine.add('DELIVER_DRINK', DeliverDrink(robot), transitions={'drink_delivered':'WAIT_AFTER_DELIVERY'})
        smach.StateMachine.add('WAIT_AFTER_DELIVERY', WaitAfterDelivery(robot), transitions={'return_to_kitchen':'RETURN_TO_KITCHEN'})
        smach.StateMachine.add('RETURN_TO_KITCHEN', ReturnToKitchen(robot), transitions={'end':'END'})

    # รัน State Machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()

