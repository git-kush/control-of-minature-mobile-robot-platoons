#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class FollowerControl(Node):
    def __init__(self):
        super().__init__("follower_control")

        # Declare and Get Parameters
        self.declare_parameter("leader_odom_topic", "/tb0/odom")   # The leader's odometry
        self.declare_parameter("follower_odom_topic", "/tb1/odom") # Follower's own odometry
        self.declare_parameter("follower_cmd_vel_topic", "/tb1/cmd_vel")  # Where to publish velocity
        self.declare_parameter("desired_distance", 1.0)  # Distance to maintain

        self.leader_odom_topic = self.get_parameter("leader_odom_topic").value
        self.follower_odom_topic = self.get_parameter("follower_odom_topic").value
        self.follower_cmd_vel_topic = self.get_parameter("follower_cmd_vel_topic").value
        self.desired_distance = self.get_parameter("desired_distance").value

        # Subscribers (Leader's and Follower's Odometry)
        self.leader_sub = self.create_subscription(Odometry, self.leader_odom_topic, self.leader_callback, 10)
        self.follower_sub = self.create_subscription(Odometry, self.follower_odom_topic, self.follower_callback, 10)

        # Publisher (Velocity Command)
        self.cmd_vel_pub = self.create_publisher(Twist, self.follower_cmd_vel_topic, 10)

        # Store Positions & Velocities
        self.leader_x = 0.0
        self.follower_x = 0.0
        self.leader_velocity = 0.0

    def leader_callback(self, msg):
        """ Callback for leader's odometry """
        self.leader_x = msg.pose.pose.position.x  # Get leader's current X position
        self.leader_velocity = msg.twist.twist.linear.x  # Get leader's speed

    def follower_callback(self, msg):
        """ Callback for follower's odometry """
        self.follower_x = msg.pose.pose.position.x  # Get follower's current X position
        self.adjust_speed()  # Adjust follower's speed

    def adjust_speed(self):
        """ Computes and publishes velocity to maintain correct spacing """
        distance_error = self.leader_x - self.follower_x - self.desired_distance

        # Simple Proportional Control (P-Controller)
        kp = 0.6  # Tuning parameter for smooth adjustments
        control_signal = kp * distance_error

        # Collision Prevention: Stop if too close
        if distance_error < 0.2:
            control_signal = 0.0

        # Ensure Follower Doesn't Move Faster Than Leader
        max_speed = min(self.leader_velocity, 0.3)
        control_signal = max(min(control_signal, max_speed), -0.3)

        # Publish the computed velocity
        cmd_vel = Twist()
        cmd_vel.linear.x = control_signal
        self.cmd_vel_pub.publish(cmd_vel)

def main(args=None):
    rclpy.init(args=args)
    node = FollowerControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
