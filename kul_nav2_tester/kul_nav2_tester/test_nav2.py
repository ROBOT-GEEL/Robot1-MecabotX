#!/usr/bin/env python3
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import rclpy
import math

def main():
    rclpy.init()
    nav = BasicNavigator()
    
    # Wait for Nav2 to be active
    nav.waitUntilNav2Active()
    
    # Goal 2m forward with obstacle in between
    goal = PoseStamped()
    goal.header.frame_id = 'base_link'
    goal.pose.position.x = 1.0
    #goal.pose.orientation.w = 1.0
    
    nav.goToPose(goal)
    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(f'Distance remaining: {feedback.distance_remaining:.2f}m')
    
    # 1. Move 1m forward (relative to robot's current position)
    #print("Moving 1m forward...")
    #forward_pose = PoseStamped()
    #forward_pose.header.frame_id = 'base_link'
    #forward_pose.header.stamp = nav.get_clock().now().to_msg()
    #forward_pose.pose.position.x = 1.0
    #nav.goToPose(forward_pose)
    #while not nav.isTaskComplete():
    #    pass
    
    # 2. Turn 90° clockwise
    #print("Turning 90° CW...")
    #turn_pose = PoseStamped()
    #turn_pose.header.frame_id = 'base_link'
    #turn_pose.header.stamp = nav.get_clock().now().to_msg()
    #turn_pose.pose.orientation.z = -0.707  # CW rotation
    #turn_pose.pose.orientation.w = 0.707
    #nav.goToPose(turn_pose)
    #while not nav.isTaskComplete():
    #    pass
    
    # 3. Move 1m left from new orientation
    #print("Moving 1m left...")
    #left_pose = PoseStamped()
    #left_pose.header.frame_id = 'base_link'
    #left_pose.header.stamp = nav.get_clock().now().to_msg()
    #left_pose.pose.position.y = 1.0  # +Y is robot's left
    #nav.goToPose(left_pose)
    #while not nav.isTaskComplete():
    #    pass
    
    print("Test complete!")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
