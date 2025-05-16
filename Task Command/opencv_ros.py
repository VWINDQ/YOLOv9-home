#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist

def spin():
    # Initialize the ROS node
    rospy.init_node('turtlebot_spin', anonymous=True)
    
    # Create a publisher to the /cmd_vel_mux/input/teleop topic
    pub = rospy.Publisher('/cmd_vel_mux/input/teleop', Twist, queue_size=10)
    
    # Set the rate at which to send the messages
    rate = rospy.Rate(10)  # 10 Hz
    
    # Create a Twist message
    twist = Twist()
    twist.linear.x = 0.0  # No linear motion
    twist.angular.z = 0.5  # Set angular velocity for spinning
    
    rospy.loginfo("Spinning TurtleBot...")

    # Publish the Twist message continuously until the node is shut down
    while not rospy.is_shutdown():
        pub.publish(twist)
        rate.sleep()

if __name__ == '__main__':
    try:
        spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("TurtleBot Spin Node Terminated.")

