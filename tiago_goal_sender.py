#!/usr/bin/env python

import rospy
import math
from geometry_msgs.msg import Twist, Pose
from gazebo_msgs.msg import ModelStates
from tf.transformations import euler_from_quaternion

class TiagoGoalSender:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('tiago_goal_sender', anonymous=True)
        
        # --- Parameters ---
        self.goal_x = 0.0  # Desired X coordinate
        self.goal_y = -1.0  # Desired Y coordinate
        
        # --- State Variables ---
        self.current_pose = Pose()
        self.current_theta = 0.0
        self.tiago_model_index = -1 # To store the index of TIAGo in the ModelStates message
        
        # --- ROS Publishers & Subscribers ---
        # Publisher for velocity commands
        self.cmd_vel_pub = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=10)
        
        # Subscriber for robot's position from Gazebo
        self.pose_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback)
        
        # Control loop rate (10 Hz)
        self.rate = rospy.Rate(10)

    def model_states_callback(self, msg):
        """
        Callback function to get the robot's current pose from Gazebo.
        """
        # Find the index of the 'tiago' model for the first time
        if self.tiago_model_index == -1:
            try:
                self.tiago_model_index = msg.name.index('tiago')
            except ValueError:
                # TIAGo model not found in the message yet
                rospy.logwarn_once("TIAGo model not found in /gazebo/model_states. Waiting...")
                return

        # Get the pose of the TIAGo model
        self.current_pose = msg.pose[self.tiago_model_index]
        
        # Convert quaternion to Euler angles to get the yaw (rotation around Z)
        orientation_q = self.current_pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        self.current_theta = yaw

    def go_to_goal(self):
        """
        Main control loop to move the robot to the goal.
        """
        # Wait until we get the first pose update
        while self.tiago_model_index == -1 and not rospy.is_shutdown():
            rospy.loginfo("Waiting for TIAGo's position...")
            self.rate.sleep()
        
        rospy.loginfo("Starting to move towards goal: ({}, {})".format(self.goal_x, self.goal_y))
        
        vel_msg = Twist()
        
        while not rospy.is_shutdown():
            # Calculate the distance to the goal
            dist_error = math.sqrt((self.goal_x - self.current_pose.position.x)**2 + 
                                   (self.goal_y - self.current_pose.position.y)**2)
            
            # --- Stopping Condition ---
            if dist_error < 0.15: # 15 cm tolerance
                rospy.loginfo("Goal Reached!")
                break
                
            # --- Control Logic ---
            # Calculate the desired angle to the goal
            path_angle = math.atan2(self.goal_y - self.current_pose.position.y,
                                    self.goal_x - self.current_pose.position.x)
            
            # Calculate the angle error (and handle wrapping)
            angle_error = path_angle - self.current_theta
            if angle_error > math.pi:
                angle_error -= 2 * math.pi
            if angle_error < -math.pi:
                angle_error += 2 * math.pi

            # --- Proportional Controller ---
            # If the angle error is significant, prioritize rotation
            if abs(angle_error) > 0.2:
                rospy.loginfo("Correcting angle...")
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.4 * angle_error # Proportional control for rotation
            # Otherwise, move forward
            else:
                rospy.loginfo("Moving forward...")
                # Proportional control for linear speed, capped at 0.5 m/s
                vel_msg.linear.x = min(0.5, 0.5 * dist_error) 
                vel_msg.angular.z = 0.0

            # Publish the velocity command
            self.cmd_vel_pub.publish(vel_msg)
            
            # Sleep to maintain the loop rate
            self.rate.sleep()
            
        # Stop the robot once the goal is reached or loop is broken
        rospy.loginfo("Stopping the robot.")
        self.cmd_vel_pub.publish(Twist()) # Publish an empty Twist to stop

if __name__ == '__main__':
    try:
        controller = TiagoGoalSender()
        # You can change the goal here or by modifying the parameters in the __init__ method
        # controller.goal_x = 3.0
        # controller.goal_y = -1.5
        controller.go_to_goal()
    except rospy.ROSInterruptException:
        pass