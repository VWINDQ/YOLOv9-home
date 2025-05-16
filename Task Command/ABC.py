#!/usr/bin/env python3

import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib import SimpleActionClient
from gtts import gTTS
import os
from std_msgs.msg import String
# Helper function for speech
def speak(text):
    tts = gTTS(text=text, lang='en')  # English language
    tts.save("/tmp/response.mp3")
    os.system("mpg321 /tmp/response.mp3")

# State: PickupBag
class PickupBag(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['picked_up', 'failed'])
        self.arduino_pub = rospy.Publisher('Test', String, queue_size=10)  # Assumed topic for the arm

    def execute(self, userdata):
        rospy.loginfo('Requesting pickup...')
        speak('I will pick up the bag now.')

        self.arduino_pub.publish("B")  # Initial pickup action
        rospy.sleep(6) 
        self.arduino_pub.publish("D")  # Arm move to pickup
        rospy.sleep(3) 
        self.arduino_pub.publish("C")  # Close gripper
        rospy.sleep(3) 
        self.arduino_pub.publish("E")  # Retract arm
        rospy.sleep(3)

        rospy.loginfo("Pickup completed.")
        speak('I have successfully picked up the bag.')

        return 'picked_up'

# State: GoToBedroom
class GoToBedroom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()
        
    def scan_for_people(self):
        """Rotate the robot 360 degrees to simulate scanning for people."""
        speak("Scanning for people.")  # Announce scanning

        twist = Twist()
        twist.angular.z = 0.5  # Set rotation speed (positive for counterclockwise)

        # Rotate for approximately one full rotation (adjust time as needed)
        scan_duration = rospy.Duration(7.0)  # Rotate for 7 seconds (adjust based on speed)
        start_time = rospy.Time.now()

        rate = rospy.Rate(10)  # 10Hz rate
        while rospy.Time.now() - start_time < scan_duration:
            self.twist_pub.publish(twist)  # Publish the twist to rotate the robot
            rate.sleep()

        # Stop the robot after rotating
        self.twist_pub.publish(Twist())  # Stop the robot
        rospy.sleep(1)  # Give time for the robot to stop

        rospy.loginfo("Scan completed.")  # Log the scan completion
        

    def execute(self, userdata):
        speak("Heading to the bedroom.")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Define the position of the bedroom
        goal.target_pose.pose.position.x = 0.61  # Coordinates of the bedroom
        goal.target_pose.pose.position.y = -3.68
        goal.target_pose.pose.orientation.w = 0.00638
        
        self.client.send_goal(goal)
        self.client.wait_for_result()
        
        if self.client.get_result():
            return 'arrived'
        else:
            return 'failed'

# State: AnnounceUnconscious
class AnnounceUnconscious(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['announced'])
        rospy.sleep(3)
        
    def execute(self, userdata):
        # Speak that an unconscious person was found in the bedroom
        speak("I found someone unconscious in the bedroom.")
        return 'announced'

# State: ReturnToKitchen
class ReturnToKitchen(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        speak("Returning to the kitchen to inform the father.")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Define the position of the kitchen
        goal.target_pose.pose.position.x = -0.864  # Coordinates of the kitchen
        goal.target_pose.pose.position.y = 0.223
        goal.target_pose.pose.orientation.w = 0.00247

        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            # When reaching the kitchen, the robot will say "I found someone unconscious"
            speak("c")
            return 'arrived'
        else:
            return 'failed'

# State: LeadFatherToBedroom
class LeadFatherToBedroom(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['arrived', 'failed'])
        self.client = SimpleActionClient('move_base', MoveBaseAction)
        self.client.wait_for_server()

    def execute(self, userdata):
        speak("Please follow me, I will lead you to the bedroom.")
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()

        # Lead the father back to the bedroom
        goal.target_pose.pose.position.x = 0.61  # Coordinates of the bedroom
        goal.target_pose.pose.position.y = -3.68
        goal.target_pose.pose.orientation.w = 0.00638

        self.client.send_goal(goal)
        self.client.wait_for_result()

        if self.client.get_result():
            speak("We have arrived at the bedroom. Please check if this person is safe.")
            return 'arrived'
        else:
            return 'failed'

# Main function
def main():
    rospy.init_node('robot_scenario')

    # Create the state machine
    sm = smach.StateMachine(outcomes=['mission_completed'])
    
    # Add states to the state machine
    with sm:
        smach.StateMachine.add('PICKUP_BAG', PickupBag(), transitions={'picked_up':'GO_TO_BEDROOM', 'failed':'mission_completed'})
        smach.StateMachine.add('GO_TO_BEDROOM', GoToBedroom(), transitions={'arrived':'ANNOUNCE_UNCONSCIOUS', 'failed':'mission_completed'})
        smach.StateMachine.add('ANNOUNCE_UNCONSCIOUS', AnnounceUnconscious(), transitions={'announced':'RETURN_TO_KITCHEN'})
        smach.StateMachine.add('RETURN_TO_KITCHEN', ReturnToKitchen(), transitions={'arrived':'LEAD_FATHER_TO_BEDROOM', 'failed':'mission_completed'})
        smach.StateMachine.add('LEAD_FATHER_TO_BEDROOM', LeadFatherToBedroom(), transitions={'arrived':'mission_completed', 'failed':'mission_completed'})

    # Execute the state machine
    outcome = sm.execute()

if __name__ == '__main__':
    main()
