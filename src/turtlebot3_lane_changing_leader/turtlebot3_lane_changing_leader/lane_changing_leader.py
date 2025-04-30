#!/usr/bin/env python3

from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_srvs.srv import SetBool
import matplotlib.pyplot as plt
import numpy as np
import math

OBSTACLE_SPEED = 0.2
SAMPLE_TIME = 0.05


class APFController(Node):
    def __init__(self):
        super().__init__('apf_controller')

        # Declare parameters
        self.declare_parameter('initial_speed', 1.0)
        self.initial_speed = self.get_parameter('initial_speed').value
        self.get_logger().info(f"Initial speed set to: {self.initial_speed}")

        # Motion enable flag - starts disabled
        self.motion_enabled = False

        self.overtaking = False
        self.pos_x = 11
        self.pos_y = 0
        self.orientation = 3.14159

        self.obstacle_pos = {
            "x": 8.0,
            "y": 0.0
        }
        self.virtual_goal = {
            "x": 0.0,
            "y": 0.0
        }

        # Publishers and subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/tb0/cmd_vel', 10)
        self.subscription = self.create_subscription(
            LaserScan,
            '/tb0/scan',
            self.scan_callback,
            10
        )
        self.pose_subscription = self.create_subscription(
            Odometry,
            '/tb0/odom',
            self.pose_callback,
            10
        )

        # Service to enable/disable motion
        self.srv = self.create_service(SetBool, 'enable_motion', self.enable_motion_callback)
        self.get_logger().info("Motion control service created. Use 'ros2 service call /enable_motion std_srvs/srv/SetBool \"data: true\"' to start moving")

        # Initialize scan data
        self.Range = [0] * 61  # number of entries

        # Wait briefly for scan data
        self.create_timer(SAMPLE_TIME, self.timer_callback)

        # For visualization
        self.i = 1
        self.initGraph()

    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        # Extract yaw from quaternion
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def enable_motion_callback(self, request, response):
        """Callback for the enable_motion service"""
        self.motion_enabled = request.data
        status = "enabled" if self.motion_enabled else "disabled"
        self.get_logger().info(f"Motion {status}")
        response.success = True
        response.message = f"Motion {status}"
        return response

    def pose_callback(self, msg):
        self.pos_x = msg.pose.pose.position.x
        self.pos_y = msg.pose.pose.position.y
        self.orientation = self.quaternion_to_yaw(msg.pose.pose.orientation)

    def scan_callback(self, msg):
        n = 61  # number of entries
        self.Range = [0] * n
        for i in range(-(n-1)//2, (n+1)//2):
            if i >= 0:
                self.Range[i+(n-1)//2] = msg.ranges[i]
            else:
                self.Range[i+(n-1)//2] = msg.ranges[len(msg.ranges)+i]

    def timer_callback(self):
        self.Loop()

    def PlotData(self):
        self.x = [0] * len(self.Range)
        self.y = [0] * len(self.Range)

        for i in range(len(self.Range)):
            angle = (i - (len(self.Range)-1)//2) * np.pi/180
            self.x[i] = -self.Range[i] * np.sin(angle)
            self.y[i] = self.Range[i] * np.cos(angle)

    def initGraph(self):
        self.PlotData()
        self.Forces()

        # Interactive plotting
        plt.ion()

        self.fig = plt.figure(figsize=(12, 6))
        self.ax = self.fig.add_subplot(121)
        self.ax2 = self.fig.add_subplot(122)

        self.ax.set_ylim(-1, 3)
        self.ax.set_xlim(-1, 1)
        self.ax2.set_ylim(300, -300)
        self.ax2.set_xlim(-300, 300)

        plt.show()

        self.scat1 = self.ax.scatter(self.x, self.y)
        self.plotForce = self.ax2.arrow(0, 0, (300 - (self.netFy))/100, self.netFx, width=0.5)

    def Forces(self):
        self.fx = [0] * len(self.Range)
        self.fy = [0] * len(self.Range)
        self.netFx = 0
        self.netFy = 0
        self.size = [0] * len(self.Range)

        for i in range(len(self.Range)):
            # Calculate forces with safety checks
            if abs(self.x[i]) > 0.1:
                self.fx[i] = 1 / self.x[i]
            else:
                self.fx[i] = 0

            if abs(self.y[i]) > 0.1:
                self.fy[i] = 1 / self.y[i]
            else:
                self.fy[i] = 0

        # Calculate net forces
        self.netFx = sum(self.fx)
        self.netFy = sum(self.fy)

        if(self.overtaking):
            dx = self.virtual_goal['x'] - self.pos_x
            dy = self.virtual_goal['y'] - self.pos_y
            distance = math.sqrt(dx**2 + dy**2)
            if(distance < 0.2):
                self.overtaking = False
                self.virtual_goal = {
                    "x": 0.0,
                    "y": 0.0
                }
            else:
                bearing = math.atan2(dy, dx)
                heading_err = self.normalize_angle(bearing - self.orientation)
                self.netFx += 25*heading_err # rotational
                self.netFy -= 5*distance # translational
                self.get_logger().info(f"heading_err: {2*heading_err}, distance: {2*distance}")

        # Calculate force magnitudes
        for i in range(len(self.Range)):
            self.size[i] = (self.fx[i]**2 + self.fy[i]**2)**0.5

    def Motion(self):
        # Create velocity command
        move = Twist()

        # Check if motion is enabled
        if not self.motion_enabled:
            # Return zero velocity if motion is disabled
            return move

        # Use the initial_speed parameter to scale the linear velocity
        base_linear_x = (100 - (self.netFy)) / 100
        move.linear.x = base_linear_x * self.initial_speed

        # Angular velocity remains the same
        move.angular.z = self.netFx / 300

        self.get_logger().info(f"Linear: {move.linear.x:.2f}, Angular: {move.angular.z:.2f}")

        return move

    def Loop(self):
        try:
            self.obstacle_pos['x'] -= OBSTACLE_SPEED*SAMPLE_TIME

            if(not self.overtaking and (0.1 < self.obstacle_pos['x'] - self.pos_x < 0.3)):
                self.overtaking = True
                self.virtual_goal = {
                    "x": self.obstacle_pos['x'] - 6,
                    "y": self.obstacle_pos['y']
                }
            self.PlotData()
            self.Forces()


            # Update visualization
            self.ax.cla()
            self.ax.set_ylim(-1, 3)
            self.ax.set_xlim(-1, 1)
            self.scat1 = self.ax.scatter(self.x, self.y)

            self.ax2.cla()
            self.ax2.set_ylim(300, -300)
            self.ax2.set_xlim(-300, 300)
            self.plotForce = self.ax2.arrow(0, 0, -self.netFx, self.netFy, width=5)

            self.fig.canvas.draw()
            self.fig.canvas.flush_events()

            # Calculate and publish velocity
            velocities = self.Motion()
            self.cmd_vel_pub.publish(velocities)

            status = "ENABLED" if self.motion_enabled else "DISABLED"
            self.get_logger().info(f"Motion {status} | Fx: {-self.netFx:.2f}, Fy: {-self.netFy:.2f}")

        except Exception as e:
            self.get_logger().error(f"Error in Loop: {e}")


def main(args=None):
    rclpy.init(args=args)
    apf_controller = APFController()

    try:
        rclpy.spin(apf_controller)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot before shutting down
        stop_msg = Twist()
        apf_controller.cmd_vel_pub.publish(stop_msg)
        apf_controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
