#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
import math

class ObstacleMover(Node):
    def __init__(self):
        super().__init__('obstacle_mover')

        # Parameters
        self.declare_parameter('obstacle_name', 'obstacle3')
        self.declare_parameter('link_name', 'link')
        self.declare_parameter('speed', 0.15)
        # Service client for direct state control
        self.set_state_client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        # # Force publisher (alternative method)
        # self.force_pub = self.create_publisher(
        #     Wrench,
        #     '/apply_force',
        #     10
        # )

        # Timer for continuous control
        self.timer = self.create_timer(
            0.1,  # 10Hz
            self.move_obstacle
        )

        self.get_logger().info("Obstacle mover node started")

    def move_obstacle(self):
        """Apply movement via either force or direct state setting"""
        # method = "state"  # Change to "state" for direct position control

        # if method == "force":
        #     # Method 1: Apply force
        #     wrench = Wrench()
        #     wrench.force.x = self.get_parameter('force_x').value
        #     self.force_pub.publish(wrench)

        # else:
            # Method 2: Direct state control (no physics)
        req = SetEntityState.Request()
        state = EntityState()
        state.name = f"{self.get_parameter('obstacle_name').value}::{self.get_parameter('link_name').value}"
        # state.pose.position.x = 5.0 + math.sin(self.get_clock().now().nanoseconds/1e9)  # Example motion
        state.twist.linear.x = 0.15  # Constant speed

        req.state = state
        self.set_state_client.call_async(req)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleMover()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
