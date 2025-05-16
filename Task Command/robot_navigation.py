#!/usr/bin/env python3
import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String

def move_to_goal(target_position):
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    goal = MoveBaseGoal()
    goal.target_pose.header.frame_id = "base_link"
    goal.target_pose.header.stamp = rospy.Time.now()

    goal.target_pose.pose.position.x = target_position[0]
    goal.target_pose.pose.position.y = target_position[1]
    goal.target_pose.pose.orientation.w = 1.0

    client.send_goal(goal)
    client.wait_for_result()

def pointing_callback(msg):
    target_position = eval(msg.data)
    move_to_goal(target_position)

def main():
    rospy.init_node('robot_navigation')
    rospy.Subscriber('/pointing_target', String, pointing_callback)

    rospy.spin()

if __name__ == '__main__':
    main()

