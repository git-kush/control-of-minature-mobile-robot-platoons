#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import time

class SimpleFollowerControl(Node):
    def __init__(self):
        super().__init__("simple_follower_control")
        
        # Parameters
        self.declare_parameter("leader_odom_topic", "/tb0/odom")
        self.declare_parameter("follower_odom_topic", "/tb1/odom")
        self.declare_parameter("follower_cmd_vel_topic", "/tb1/cmd_vel")
        self.declare_parameter("desired_distance", 1.0)
        
        self.leader_odom_topic = self.get_parameter("leader_odom_topic").value
        self.follower_odom_topic = self.get_parameter("follower_odom_topic").value
        self.follower_cmd_vel_topic = self.get_parameter("follower_cmd_vel_topic").value
        self.desired_distance = self.get_parameter("desired_distance").value
        
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
        
        # Control constants
        self.linear_gain = 0.5
        self.angular_gain = 1.2
        self.distance_tolerance = 0.1  # meters
        
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
        if self.leader_pos is None or self.follower_pos is None:
            return
        
        # Calculate vector from follower to leader
        dx = self.leader_pos.x - self.follower_pos.x
        dy = self.leader_pos.y - self.follower_pos.y
        
        # Calculate actual distance and bearing to leader
        distance = math.sqrt(dx*dx + dy*dy)
        bearing = math.atan2(dy, dx)
        
        # Calculate distance error (positive means too far, negative means too close)
        distance_error = distance - self.desired_distance
        
        # Calculate heading error (how much to turn to face the leader)
        heading_error = self.normalize_angle(bearing - self.follower_orientation)
        
        # Create control message
        cmd = Twist()
        
        # Angular control: Turn to face the leader
        cmd.angular.z = self.angular_gain * heading_error
        
        # Linear control: Move toward or away from leader to maintain distance
        # Only move forward if mostly facing the leader (within ~45 degrees)
        if abs(heading_error) < 0.8:  # ~45 degrees
            # Speed is proportional to distance error
            cmd.linear.x = self.linear_gain * distance_error
            
            # Add a component of leader's speed to maintain formation
            if distance_error > -self.distance_tolerance and abs(heading_error) < 0.3:
                leader_speed = math.sqrt(self.leader_linear_vel.x**2 + self.leader_linear_vel.y**2)
                # Only consider leader's velocity if it's moving
                if leader_speed > 0.05:
                    leader_heading = math.atan2(self.leader_linear_vel.y, self.leader_linear_vel.x)
                    # Calculate how aligned the follower is with leader's motion
                    alignment = math.cos(self.normalize_angle(leader_heading - self.follower_orientation))
                    # Add leader velocity component scaled by alignment
                    cmd.linear.x += 0.8 * leader_speed * max(0, alignment)
        else:
            # If not facing the leader, prioritize turning
            cmd.linear.x = 0.0
        
        # Safety limits
        max_linear =  0.42     #0.22  # m/s, safe max for TurtleBot3
        max_angular = 2.0   # rad/s
        
        # Apply speed limits
        cmd.linear.x = max(min(cmd.linear.x, max_linear), -max_linear)
        cmd.angular.z = max(min(cmd.angular.z, max_angular), -max_angular)
        
        # Emergency stop if too close
        if distance < 0.5:
            cmd.linear.x = min(cmd.linear.x, 0.0)  # Only allow backing up
        
        # Publish command
        self.cmd_vel_pub.publish(cmd)
        
        # Logging (once per second)
        if int(time.time()) % 5 == 0:
            self.get_logger().info(
                f"Distance: {distance:.2f}m (target: {self.desired_distance:.2f}m), "
                f"Heading error: {math.degrees(heading_error):.1f}°, "
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