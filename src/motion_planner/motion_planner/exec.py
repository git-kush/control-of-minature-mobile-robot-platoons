#!/usr/bin/env python3
from os import F_OK
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
# import numpy as np
class SimpleFollowerControl(Node):
    def __init__(self):
        super().__init__("simple_follower_control")

        # Parameters
        self.declare_parameter("leader_odom_topic", "/tb0/odom")
        self.declare_parameter("follower_odom_topic", "/tb1/odom")
        self.declare_parameter("follower_cmd_vel_topic", "/tb1/cmd_vel")
        self.declare_parameter("predecessor_odom_topic", None)
        self.declare_parameter("successor_odom_topic", None)
        self.declare_parameter("desired_distance", 1.0)
        self.declare_parameter("number", 0)

        self.leader_odom_topic = self.get_parameter("leader_odom_topic").value
        self.follower_odom_topic = self.get_parameter("follower_odom_topic").value
        self.follower_cmd_vel_topic = self.get_parameter("follower_cmd_vel_topic").value
        self.predecessor_odom_topic = self.get_parameter("predecessor_odom_topic").value
        self.successor_odom_topic = self.get_parameter("successor_odom_topic").value
        self.desired_distance = self.get_parameter("desired_distance").value
        self.number = self.get_parameter("number").value

        self.is_leader = self.number == 0 or self.predecessor_odom_topic is None
        self.is_last = self.successor_odom_topic is None

        self.get_logger().info(f"Starting follower with desired distance: {self.desired_distance}")

        # Create subscribers and publisher
        self.leader_sub = self.create_subscription(
            Odometry,
            self.leader_odom_topic,
            self.leader_callback,
            10
        )

        self.follower_sub = self.create_subscription(
            Odometry,
            self.follower_odom_topic,
            self.follower_callback,
            10
        )
        if not self.is_last:
            self.successor_sub = self.create_subscription(
                Odometry,
                self.successor_odom_topic,
                self.successor_callback,
                10
            )

        self.predecessor_sub = self.create_subscription(
            Odometry,
            self.predecessor_odom_topic,
            self.predecessor_callback,
            10
        )

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.follower_cmd_vel_topic,
            10
        )

        # Initialize state variables
        self.leader_pos = None
        self.leader_orientation = None
        self.leader_linear_vel = None
        self.leader_angular_vel = None

        self.follower_pos = None
        self.follower_orientation = None
        self.follower_linear_vel = None
        self.follower_angular_vel = None

        self.predecessor_pos = None
        self.predecessor_orientation = None
        self.predecessor_linear_vel = None
        self.predecessor_angular_vel = None

        self.successor_pos = None
        self.successor_orientation = None
        self.successor_linear_vel = None
        self.successor_angular_vel = None

        # Control constants
        self.linear_gain_l = 0.1
        self.linear_gain_p = 0.4
        self.linear_gain_s = 0.2
        self.angular_gain_l = 0.4
        self.angular_gain_p = 0.8
        # self.distance_tolerance = 0.1  # meters
        # no need


        # if self.successor_odom_topic is None:
        #     self.get_logger().info(f"IsLast number: {self.number}")

        # Create control timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control

        self.get_logger().info("Follower controller initialized and ready")

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def leader_callback(self, msg):
        """Process the leader's odometry data"""
        self.leader_pos = msg.pose.pose.position
        self.leader_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.leader_linear_vel = msg.twist.twist.linear
        self.leader_angular_vel = msg.twist.twist.angular

    def follower_callback(self, msg):
        """Process the follower's odometry data"""
        self.follower_pos = msg.pose.pose.position
        self.follower_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.follower_linear_vel = msg.twist.twist.linear
        self.follower_angular_vel = msg.twist.twist.angular

    def predecessor_callback(self, msg):
        """Process the predecessor's odometry data"""
        self.predecessor_pos = msg.pose.pose.position
        self.predecessor_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.predecessor_linear_vel = msg.twist.twist.linear
        self.predecessor_angular_vel = msg.twist.twist.angular

    def successor_callback(self, msg):
        """Process the successor's odometry data"""
        self.successor_pos = msg.pose.pose.position
        self.successor_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.successor_linear_vel = msg.twist.twist.linear
        self.successor_angular_vel = msg.twist.twist.angular

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def control_loop(self):
        """Main control loop that runs at fixed frequency"""
        # Check if we have received data from both robots
        # if self.leader_pos is None or self.follower_pos is None or (not self.is_last and self.successor_pos is None):
        if self.leader_pos is None or self.follower_pos is None or self.predecessor_pos is None or ((not self.is_last) and (self.successor_pos is None)):
            return

        # Calculate vector from follower to leader
        # net_force = np.array([0.0, 0.0]) # f_linear, f_rot
        f_linear = 0.0
        f_rot = 0.0


        dxl = self.leader_pos.x - self.follower_pos.x
        dyl = self.leader_pos.y - self.follower_pos.y

        dyp = self.predecessor_pos.y - self.follower_pos.y
        dxp = self.predecessor_pos.x - self.follower_pos.x
        distance_l = math.sqrt(dxl**2 + dyl**2)
        distance_p = math.sqrt(dxp**2 + dyp**2)
        bearing_l = math.atan2(dyl, dxl)
        bearing_p = math.atan2(dyp, dxp)

        steepness_const = 1
        f_linear_l = self.linear_gain_l*math.tanh(steepness_const*(distance_l - self.desired_distance*self.number))
        f_linear_p = self.linear_gain_p*math.tanh(steepness_const*(distance_p - self.desired_distance))

        # this force will accelerate or deccelerate the bot assumign it has the right heading
        f_linear = f_linear_l + f_linear_p
        heading_error_l = self.normalize_angle(bearing_l - self.follower_orientation)
        f_rot_l = self.angular_gain_l*heading_error_l
        heading_error_p = self.normalize_angle(bearing_p - self.follower_orientation)
        f_rot_p = self.angular_gain_p*heading_error_p
        # net_force += np.array([force_l, self.angular_gain*heading_error_l])
        f_rot = f_rot_l + f_rot_p

        if(not self.is_last):
            dxs = self.successor_pos.x - self.follower_pos.x
            dys = self.successor_pos.y - self.follower_pos.y
            distance_s = math.sqrt(dxs**2 + dys**2)
            f_linear_s = self.linear_gain_s*math.tanh(steepness_const*(distance_s - self.desired_distance))
            f_linear -= f_linear_s

        # Create control message
        cmd = Twist()

        # Angular control: Turn to face the leader
        # cmd.angular.z = 0.92*f_rot + 0.08*self.follower_angular_vel.z
        # the idea behind this is to let f_rot act as a force ~ acceleration and be added to velocity instead of directly equating it to angular velocity
        # but this introduces a lag in the system

        cmd.angular.z = f_rot

        # Linear control: Move toward or away from leader to maintain distance
        # Only move forward if mostly facing the leader (within ~45 degrees)
        if abs(heading_error_l) < 0.8:  # ~45 degrees
            # Speed is proportional to distance error
            cmd.linear.x = f_linear + self.follower_linear_vel.x
        else:
            # If not facing the leader, prioritize turning
            cmd.linear.x = 0.0

        # Safety limits
        max_linear =  0.42     #0.22  # m/s, safe max for TurtleBot3
        max_angular = 2.0   # rad/s

        # Apply speed limits
        cmd.linear.x = max(min(cmd.linear.x, max_linear), 0.0)
        # minimum speed is 0, does not go back
        cmd.angular.z = max(min(cmd.angular.z, max_angular), -max_angular)

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Logging (once per second)
        if int(time.time()) % 5 == 0:
            self.get_logger().info(
                f"number:{self.number}"
                f"Cmd: linear={cmd.linear.x:.2f}, angular={cmd.angular.z:.2f}"
            )

def main(args=None):
    rclpy.init(args=args)
    follower = SimpleFollowerControl()
    try:
        rclpy.spin(follower)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_cmd = Twist()
        follower.cmd_vel_pub.publish(stop_cmd)
        follower.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
