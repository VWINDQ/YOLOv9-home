#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gtts import gTTS
import os

class FollowPerson:
    def __init__(self):
        rospy.init_node('follow_person_node')

        self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.laser_callback)
        self.twist_pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
        self.person_detected = False
        self.min_distance = 0.5  # Minimum distance to keep from the person
        self.max_distance = 1.5  # Maximum distance to consider as a person
        self.front_angle_range = 30  # Range of angles to consider in degrees

    def laser_callback(self, msg):
        min_distance = float('inf')
        min_index = -1
        half_range = len(msg.ranges) // 2  # Halfway index for front

        # Calculate the angle range in terms of array index
        angle_indices = int(self.front_angle_range / msg.angle_increment * (3.14159 / 180.0))

        # Limit scanning to the front range of the robot
        start_index = half_range - angle_indices
        end_index = half_range + angle_indices

        for i in range(start_index, end_index):
            distance = msg.ranges[i]
            if self.min_distance < distance < self.max_distance and distance < min_distance:
                min_distance = distance
                min_index = i

        if min_index != -1:
            self.person_detected = True
            angle = (min_index - len(msg.ranges) / 2) * msg.angle_increment
            twist = Twist()
            twist.linear.x = 0.2  # Move forward
            twist.angular.z = -angle  # Turn towards the person
            self.twist_pub.publish(twist)
        else:
            self.person_detected = False
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.twist_pub.publish(twist)

    def speak(self, text):
        tts = gTTS(text=text, lang='en')
        tts.save("/tmp/speak.mp3")
        os.system("mpg321 /tmp/speak.mp3")

    def run(self):
        rospy.loginfo('Following the person...')
        self.speak('I am following you.')

        rospy.spin()  # Keep the node running

if __name__ == '__main__':
    follower = FollowPerson()
    follower.run()

