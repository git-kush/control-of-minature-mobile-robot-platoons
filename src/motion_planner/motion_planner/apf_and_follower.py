#!/usr/bin/env python3
from matplotlib.font_manager import Number
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time
import numpy as np

class SimpleFollowerControl(Node):
    def __init__(self):
        super().__init__("motion_planner")

        # Parameters

        self.declare_parameter('initial_speed', 0.0)
        #initially at rest, all expect the leader

        self.linear_x = self.get_parameter('initial_speed').value

        self.declare_parameter("leader_odom_topic", "/tb0/cmd/odom")
        # this is always true

        self.declare_parameter("successor_odom_topic", "null")
        #default value, the one leading the current vehicle

        self.declare_parameter("predecessor_odom_topic", "null")
        #default value, the one following the current vehicle

        self.declare_parameter("odom_topic", "/tb1/odom")
        #default value, the current vehicle's odom

        self.declare_parameter("cmd_vel_topic", "/tb1/cmd_vel")
        #default value, the current vehicle's cmd_vel

        #desired distance between the predecessor and successor
        self.declare_parameter("desired_distance", 1.0)

        self.declare_parameter("number", 1)

        self.obstacle_pos = {
            'x': 8.0,
            'y': 0.0
        }
        self.obstacle_vel = {
            'x': -0.2,
            'y': 0.0
        }

        #parameters
        self.leader_force_multiplier = 0.2
        self.pred_force_multiplier = 0.4
        self.succ_force_multiplier = 0.2
        self.obstacle_force_multiplier = 1
        self.leader_angular_gain = 0.4
        self.pred_angular_gain = 0.8

        self.leader_odom_topic = self.get_parameter("leader_odom_topic").value
        self.predecessor_odom_topic = self.get_parameter("predecessor_odom_topic").value
        self.successor_odom_topic = self.get_parameter("successor_odom_topic").value
        self.odom_topic = self.get_parameter("odom_topic").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.desired_distance = self.get_parameter("desired_distance").value
        self.number = self.get_parameter("number").value
        self.desired_distance_leader = self.desired_distance*self.number

        self.get_logger().info(f"Starting follower with desired distance: {self.desired_distance}")

        self.is_leader = self.number == 0 or self.predecessor_odom_topic == "null"
        self.is_last = self.successor_odom_topic == "null"

        # Create subscribers and publisher
        if not self.is_leader:
            self.leader_sub = self.create_subscription(
                Odometry,
                self.leader_odom_topic,
                self.leader_callback,
                10
            )
            self.successor_sub = self.create_subscription(
                Odometry,
                self.successor_odom_topic,
                self.successor_callback,
                10
            )
        if not self.is_last:
            self.predecessor_sub = self.create_subscription(
                Odometry,
                self.predecessor_odom_topic,
                self.predecessor_callback,
                10
            )

        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            self.cmd_vel_topic,
            10
        )

        #variables
        self.leader_pos = None
        self.leader_orientation = None
        self.leader_linear_vel = None
        self.leader_angular_vel = None

        self.pred_pos = None
        self.pred_orientation = None
        self.pred_linear_vel = None
        self.pred_angular_vel = None

        self.succ_pos = None
        self.succ_orientation = None
        self.succ_linear_vel = None
        self.succ_angular_vel = None

        self.pos = None
        self.orientation = None
        self.linear_vel = None
        self.angular_vel = None

        # Create control timer
        self.timer = self.create_timer(0.05, self.control_loop)  # 20Hz control

        self.get_logger().info("Follower controller initialized and ready")

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
            """Normalize angle to [-pi, pi]"""
            while angle > math.pi:
                angle -= 2.0 * math.pi
            while angle < -math.pi:
                angle += 2.0 * math.pi
            return angle

    def leader_callback(self, msg):
        """Process the leader's odometry data"""
        self.leader_pos = msg.pose.pose.position
        self.leader_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.leader_linear_vel = msg.twist.twist.linear
        self.leader_angular_vel = msg.twist.twist.angular

    def predecessor_callback(self, msg):
        """Process the predecessor's odometry data"""
        self.pred_pos = msg.pose.pose.position
        self.pred_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.pred_linear_vel = msg.twist.twist.linear
        self.pred_angular_vel = msg.twist.twist.angular

    def successor_callback(self, msg):
        """Process the successor's odometry data"""
        self.succ_pos = msg.pose.pose.position
        self.succ_orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.succ_linear_vel = msg.twist.twist.linear
        self.succ_angular_vel = msg.twist.twist.angular

    def odom_callback(self, msg):
        """Process self's odometry data"""
        self.pos = msg.pose.pose.position
        self.orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.linear_vel = msg.twist.twist.linear
        self.angular_vel = msg.twist.twist.angular

    def control_loop(self):
        """Main control loop that runs at fixed frequency"""
        # Check if we have received data from both robots
        if ((not self.is_leader and (self.leader_pos is None or self.pred_pos is None)) or self.pos is None or (not self.is_last and self.succ_pos is None)):
            return

        # Calculate vector from follower to leader

        net_force = np.array([0.0, 0.0]) # f_linear, f_rot

        if not self.is_leader:
            d_l = np.array([self.leader_pos.x - self.pos.x, self.leader_pos.y - self.pos.y])
            d_p = np.array([self.pred_pos.x - self.pos.x, self.pred_pos.y - self.pos.y])
            distance_l = np.linalg.norm(d_l)
            distance_p = np.linalg.norm(d_p)
            bearing_l = math.atan2(d_l[1], d_l[0])
            bearing_p = math.atan2(d_p[1], d_p[0])

            force_l = self.leader_force_multiplier*math.tanh(3*distance_l - self.desired_distance_leader)
            force_p = self.pred_force_multiplier*math.tanh(3*distance_p - self.desired_distance)
            # this force will accelerate or deccelerate the bot assumign it has the right heading

            heading_error_l = self.normalize_angle(bearing_l - self.orientation)
            heading_error_p = self.normalize_angle(bearing_p - self.orientation)

            net_force += [force_l+force_p, self.leader_angular_gain*heading_error_l + self.pred_angular_gain*heading_error_p]



        if not self.is_last:
            d_s = np.array([self.succ_pos.x - self.pos.x, self.succ_pos.y - self.pos.y])
            distance_s = np.linalg.norm(d_s)
            force_s = self.pred_force_multiplier*math.tanh(3*distance_s - self.desired_distance)
            net_force[0] -= force_s
            #follower does not affect the predecessor's heading

        # Calculate heading error (how much to turn to face the leader)
        # heading_error = self.normalize_angle(bearing - self.orientation)
        # heading error not needed for time being

        # Create control message
        cmd = Twist()

        # Angular control: Turn to face the leader

        cmd.angular.z = self.angular_vel + net_force[1] #(force/mass)*sampling_time => constants are absorbed in angular_gain_constants

        # Linear control: Move toward or away from leader to maintain distance
        # Only move forward if mostly facing the leader (within ~45 degrees)
        if abs(net_force[1]) < 0.8:  # ~45 degrees, change 0.8 to something else
            # Speed is proportional to distance error
            cmd.linear.x = self.linear_vel + net_force[0]

            # Add a component of leader's speed to maintain formation
            # #### idk how this works, first lets try without this, will prolly add it back later#####
            # if distance_error > -self.distance_tolerance and abs(heading_error) < 0.3:
            #     leader_speed = math.sqrt(self.leader_linear_vel.x**2 + self.leader_linear_vel.y**2)
            #     # Only consider leader's velocity if it's moving
            #     if leader_speed > 0.05:
            #         leader_heading = math.atan2(self.leader_linear_vel.y, self.leader_linear_vel.x)
            #         # Calculate how aligned the follower is with leader's motion
            #         alignment = math.cos(self.normalize_angle(leader_heading - self.follower_orientation))
            #         # Add leader velocity component scaled by alignment
            #         cmd.linear.x += 0.8 * leader_speed * max(0, alignment)
        elif self.number == 0:
            cmd.linear.x = 0.4
        else:
            # If not facing the leader, prioritize turning
            cmd.linear.x = 0.0

        # Safety limits
        max_linear =  0.42     #0.22  # m/s, safe max for TurtleBot3
        max_angular = 2.0   # rad/s

        # Apply speed limits
        cmd.linear.x = max(min(cmd.linear.x, max_linear), 0)
        cmd.angular.z = max(min(cmd.angular.z, max_angular), -max_angular)

        # # Emergency stop if too close
        # if distance < 0.5:
        #     cmd.linear.x = min(cmd.linear.x, 0.0)  # Only allow backing up
        # above lines will take care of this when apf will be implemented

        # Publish command
        self.cmd_vel_pub.publish(cmd)

        # Logging (once per second)
        if int(time.time()) % 5 == 0:
            self.get_logger().info("working")

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
