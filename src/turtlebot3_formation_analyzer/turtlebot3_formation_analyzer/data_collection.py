#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import matplotlib.pyplot as plt
import numpy as np
import math
import time
from matplotlib.animation import FuncAnimation

class FormationDataCollector(Node):
    def __init__(self):
        super().__init__('formation_data_collector')
        
        # Create subscribers for all three robots' odometry
        self.tb0_odom_sub = self.create_subscription(
            Odometry, '/tb0/odom', self.tb0_odom_callback, 10)
        self.tb1_odom_sub = self.create_subscription(
            Odometry, '/tb1/odom', self.tb1_odom_callback, 10)
        self.tb2_odom_sub = self.create_subscription(
            Odometry, '/tb2/odom', self.tb2_odom_callback, 10)
            
        # Create subscribers for velocity commands
        self.tb0_vel_sub = self.create_subscription(
            Twist, '/tb0/cmd_vel', self.tb0_vel_callback, 10)
        self.tb1_vel_sub = self.create_subscription(
            Twist, '/tb1/cmd_vel', self.tb1_vel_callback, 10)
        self.tb2_vel_sub = self.create_subscription(
            Twist, '/tb2/cmd_vel', self.tb2_vel_callback, 10)
        
        # Initialize position and velocity data
        self.robot_positions = {
            'tb0': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'vy': 0.0},
            'tb1': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'vy': 0.0},
            'tb2': {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'vx': 0.0, 'vy': 0.0}
        }
        
        # Command velocities
        self.cmd_vel = {
            'tb0': {'vx': 0.0, 'wz': 0.0},
            'tb1': {'vx': 0.0, 'wz': 0.0}, 
            'tb2': {'vx': 0.0, 'wz': 0.0}
        }
        
        # Data storage for plotting
        self.times = []
        self.start_time = time.time()
        
        # Longitudinal and lateral distances
        self.long_dist_l1_f1 = []  # tb0 to tb1
        self.long_dist_f1_f2 = []  # tb1 to tb2
        self.lat_dist_l1_f1 = []   # tb0 to tb1
        self.lat_dist_f1_f2 = []   # tb1 to tb2
        
        # Velocity differences
        self.long_vel_diff_l1_f1 = []  # tb0 to tb1
        self.long_vel_diff_f1_f2 = []  # tb1 to tb2
        self.lat_vel_diff_l1_f1 = []   # tb0 to tb1
        self.lat_vel_diff_f1_f2 = []   # tb1 to tb2
        
        # Create timer for data processing and plotting
        self.timer = self.create_timer(0.1, self.process_data)  # 10Hz
        
        # Setup plots
        self.setup_plots()
        
        self.get_logger().info("Formation data collector initialized")
    
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle in radians"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def tb0_odom_callback(self, msg):
        """Process leader (tb0) odometry data"""
        self.robot_positions['tb0']['x'] = msg.pose.pose.position.x
        self.robot_positions['tb0']['y'] = msg.pose.pose.position.y
        self.robot_positions['tb0']['theta'] = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_positions['tb0']['vx'] = msg.twist.twist.linear.x
        self.robot_positions['tb0']['vy'] = msg.twist.twist.linear.y
    
    def tb1_odom_callback(self, msg):
        """Process follower 1 (tb1) odometry data"""
        self.robot_positions['tb1']['x'] = msg.pose.pose.position.x
        self.robot_positions['tb1']['y'] = msg.pose.pose.position.y
        self.robot_positions['tb1']['theta'] = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_positions['tb1']['vx'] = msg.twist.twist.linear.x
        self.robot_positions['tb1']['vy'] = msg.twist.twist.linear.y
    
    def tb2_odom_callback(self, msg):
        """Process follower 2 (tb2) odometry data"""
        self.robot_positions['tb2']['x'] = msg.pose.pose.position.x
        self.robot_positions['tb2']['y'] = msg.pose.pose.position.y
        self.robot_positions['tb2']['theta'] = self.quaternion_to_yaw(msg.pose.pose.orientation)
        self.robot_positions['tb2']['vx'] = msg.twist.twist.linear.x
        self.robot_positions['tb2']['vy'] = msg.twist.twist.linear.y
    
    def tb0_vel_callback(self, msg):
        """Process leader (tb0) command velocity"""
        self.cmd_vel['tb0']['vx'] = msg.linear.x
        self.cmd_vel['tb0']['wz'] = msg.angular.z
    
    def tb1_vel_callback(self, msg):
        """Process follower 1 (tb1) command velocity"""
        self.cmd_vel['tb1']['vx'] = msg.linear.x
        self.cmd_vel['tb1']['wz'] = msg.angular.z
    
    def tb2_vel_callback(self, msg):
        """Process follower 2 (tb2) command velocity"""
        self.cmd_vel['tb2']['vx'] = msg.linear.x
        self.cmd_vel['tb2']['wz'] = msg.angular.z
    
    def setup_plots(self):
        """Initialize plots"""
        plt.ion()  # Enable interactive mode
        
        self.fig, self.axs = plt.subplots(2, 2, figsize=(12, 10))
        
        # Setup subplot titles and labels
        self.axs[0, 0].set_title('Longitudinal Distance')
        self.axs[0, 0].set_xlabel('Time(s)')
        self.axs[0, 0].set_ylabel('Longitudinal distance(m)')
        
        self.axs[0, 1].set_title('Lateral Distance')
        self.axs[0, 1].set_xlabel('Time(s)')
        self.axs[0, 1].set_ylabel('Lateral distance(m)')
        
        self.axs[1, 0].set_title('Longitudinal Velocity Difference')
        self.axs[1, 0].set_xlabel('Time(s)')
        self.axs[1, 0].set_ylabel('Longitudinal velocity difference(m/s)')
        
        self.axs[1, 1].set_title('Lateral Velocity Difference')
        self.axs[1, 1].set_xlabel('Time(s)')
        self.axs[1, 1].set_ylabel('Lateral velocity difference(m/s)')
        
        # Initialize line objects for plots
        self.long_dist_l1_f1_line, = self.axs[0, 0].plot([], [], 'r-', label='Longitudinal distance between L1 and F1')
        self.long_dist_f1_f2_line, = self.axs[0, 0].plot([], [], 'b-', label='Longitudinal distance between F1 and F2')
        
        self.lat_dist_l1_f1_line, = self.axs[0, 1].plot([], [], 'r-', label='Lateral distance between L1 and F1')
        self.lat_dist_f1_f2_line, = self.axs[0, 1].plot([], [], 'b-', label='Lateral distance between F1 and F2')
        
        self.long_vel_diff_l1_f1_line, = self.axs[1, 0].plot([], [], 'r-', label='Longitudinal velocity difference between L1 and F1')
        self.long_vel_diff_f1_f2_line, = self.axs[1, 0].plot([], [], 'b-', label='Longitudinal velocity difference between F1 and F2')
        
        self.lat_vel_diff_l1_f1_line, = self.axs[1, 1].plot([], [], 'r-', label='Lateral velocity difference between L1 and F1')
        self.lat_vel_diff_f1_f2_line, = self.axs[1, 1].plot([], [], 'b-', label='Lateral velocity difference between F1 and F2')
        
        # Add legends
        for ax in self.axs.flat:
            ax.legend()
        
        plt.tight_layout()
        plt.show(block=False)
    
    def calculate_relative_distance(self, lead, follow):
        """Calculate longitudinal and lateral distance between robots in lead robot's frame"""
        # Get positions
        lead_x = self.robot_positions[lead]['x']
        lead_y = self.robot_positions[lead]['y']
        lead_theta = self.robot_positions[lead]['theta']
        
        follow_x = self.robot_positions[follow]['x']
        follow_y = self.robot_positions[follow]['y']
        
        # Calculate relative position
        dx = follow_x - lead_x
        dy = follow_y - lead_y
        
        # Transform to lead robot's coordinate frame
        cos_theta = math.cos(lead_theta)
        sin_theta = math.sin(lead_theta)
        
        # Rotation matrix applied to [dx, dy]
        x_rel = dx * cos_theta + dy * sin_theta
        y_rel = -dx * sin_theta + dy * cos_theta
        
        return x_rel, y_rel
    
    def calculate_relative_velocity(self, lead, follow):
        """Calculate longitudinal and lateral velocity difference between robots in lead robot's frame"""
        # Get velocities and orientation
        lead_vx = self.robot_positions[lead]['vx']
        lead_vy = self.robot_positions[lead]['vy']
        lead_theta = self.robot_positions[lead]['theta']
        
        follow_vx = self.robot_positions[follow]['vx']
        follow_vy = self.robot_positions[follow]['vy']
        
        # Calculate velocity difference
        dvx = follow_vx - lead_vx
        dvy = follow_vy - lead_vy
        
        # Transform to lead robot's coordinate frame
        cos_theta = math.cos(lead_theta)
        sin_theta = math.sin(lead_theta)
        
        # Rotation matrix applied to [dvx, dvy]
        vx_rel = dvx * cos_theta + dvy * sin_theta
        vy_rel = -dvx * sin_theta + dvy * cos_theta
        
        return vx_rel, vy_rel
    
    def process_data(self):
        """Calculate metrics and update plots"""
        try:
            # Calculate current time
            current_time = time.time() - self.start_time
            self.times.append(current_time)
            
            # Calculate distances
            l1_f1_x, l1_f1_y = self.calculate_relative_distance('tb0', 'tb1')
            f1_f2_x, f1_f2_y = self.calculate_relative_distance('tb1', 'tb2')
            
            # Store distance data
            self.long_dist_l1_f1.append(l1_f1_x)
            self.long_dist_f1_f2.append(f1_f2_x)
            self.lat_dist_l1_f1.append(l1_f1_y)
            self.lat_dist_f1_f2.append(f1_f2_y)
            
            # Calculate velocity differences
            l1_f1_vx, l1_f1_vy = self.calculate_relative_velocity('tb0', 'tb1')
            f1_f2_vx, f1_f2_vy = self.calculate_relative_velocity('tb1', 'tb2')
            
            # Store velocity data
            self.long_vel_diff_l1_f1.append(l1_f1_vx)
            self.long_vel_diff_f1_f2.append(f1_f2_vx)
            self.lat_vel_diff_l1_f1.append(l1_f1_vy)
            self.lat_vel_diff_f1_f2.append(f1_f2_vy)
            
            # Update plots
            self.update_plots()
            
            # Log data occasionally
            if len(self.times) % 50 == 0:  # Log every 5 seconds at 10Hz
                self.get_logger().info(
                    f"Time: {current_time:.1f}s, "
                    f"L1-F1 Long: {l1_f1_x:.2f}m, Lat: {l1_f1_y:.2f}m, "
                    f"F1-F2 Long: {f1_f2_x:.2f}m, Lat: {f1_f2_y:.2f}m"
                )
                
                # Save data to files periodically
                self.save_data_to_file()
                
        except Exception as e:
            self.get_logger().error(f"Error in process_data: {e}")
    
    def update_plots(self):
        """Update the plots with current data"""
        try:
            # Update distance plots
            self.long_dist_l1_f1_line.set_data(self.times, self.long_dist_l1_f1)
            self.long_dist_f1_f2_line.set_data(self.times, self.long_dist_f1_f2)
            
            self.lat_dist_l1_f1_line.set_data(self.times, self.lat_dist_l1_f1)
            self.lat_dist_f1_f2_line.set_data(self.times, self.lat_dist_f1_f2)
            
            # Update velocity difference plots
            self.long_vel_diff_l1_f1_line.set_data(self.times, self.long_vel_diff_l1_f1)
            self.long_vel_diff_f1_f2_line.set_data(self.times, self.long_vel_diff_f1_f2)
            
            self.lat_vel_diff_l1_f1_line.set_data(self.times, self.lat_vel_diff_l1_f1)
            self.lat_vel_diff_f1_f2_line.set_data(self.times, self.lat_vel_diff_f1_f2)
            
            # Adjust limits for all subplots
            for ax in self.axs.flat:
                ax.relim()
                ax.autoscale_view()
            
            # Draw the updated plots
            self.fig.canvas.draw_idle()
            self.fig.canvas.flush_events()
            
        except Exception as e:
            self.get_logger().error(f"Error in update_plots: {e}")
    
    def save_data_to_file(self):
        """Save collected data to CSV files"""
        try:
            # Create numpy arrays of the data
            data_array = np.column_stack((
                self.times,
                self.long_dist_l1_f1, self.long_dist_f1_f2,
                self.lat_dist_l1_f1, self.lat_dist_f1_f2,
                self.long_vel_diff_l1_f1, self.long_vel_diff_f1_f2,
                self.lat_vel_diff_l1_f1, self.lat_vel_diff_f1_f2
            ))
            
            # Save to CSV file
            np.savetxt(
                'formation_data.csv',
                data_array,
                delimiter=',',
                header='Time,Long_Dist_L1_F1,Long_Dist_F1_F2,Lat_Dist_L1_F1,Lat_Dist_F1_F2,Long_Vel_Diff_L1_F1,Long_Vel_Diff_F1_F2,Lat_Vel_Diff_L1_F1,Lat_Vel_Diff_F1_F2',
                comments=''
            )
            
            self.get_logger().info("Data saved to formation_data.csv")
            
        except Exception as e:
            self.get_logger().error(f"Error saving data to file: {e}")
    
    def generate_plots_from_file(self, filename='formation_data.csv'):
        """Generate static plots from saved data file"""
        try:
            # Load data from CSV
            data = np.loadtxt(filename, delimiter=',', skiprows=1)
            
            times = data[:, 0]
            long_dist_l1_f1 = data[:, 1]
            long_dist_f1_f2 = data[:, 2]
            lat_dist_l1_f1 = data[:, 3]
            lat_dist_f1_f2 = data[:, 4]
            long_vel_diff_l1_f1 = data[:, 5]
            long_vel_diff_f1_f2 = data[:, 6]
            lat_vel_diff_l1_f1 = data[:, 7]
            lat_vel_diff_f1_f2 = data[:, 8]
            
            # Create figure with 2x2 subplots
            fig, axs = plt.subplots(2, 2, figsize=(12, 10))
            
            # Plot longitudinal distance
            axs[0, 0].plot(times, long_dist_l1_f1, 'r-', label='Longitudinal distance between L1 and F1')
            axs[0, 0].plot(times, long_dist_f1_f2, 'b-', label='Longitudinal distance between F1 and F2')
            axs[0, 0].set_title('(a) Longitudinal distance')
            axs[0, 0].set_xlabel('Time(s)')
            axs[0, 0].set_ylabel('Longitudinal distance(m)')
            axs[0, 0].legend()
            
            # Plot lateral distance
            axs[0, 1].plot(times, lat_dist_l1_f1, 'r-', label='Lateral distance between L1 and F1')
            axs[0, 1].plot(times, lat_dist_f1_f2, 'b-', label='Lateral distance between F1 and F2')
            axs[0, 1].set_title('(b) Lateral distance')
            axs[0, 1].set_xlabel('Time(s)')
            axs[0, 1].set_ylabel('Lateral distance(m)')
            axs[0, 1].legend()
            
            # Plot longitudinal velocity difference
            axs[1, 0].plot(times, long_vel_diff_l1_f1, 'r-', label='Longitudinal velocity difference between L1 and F1')
            axs[1, 0].plot(times, long_vel_diff_f1_f2, 'b-', label='Longitudinal velocity difference between F1 and F2')
            axs[1, 0].set_title('(a) Longitudinal velocity difference')
            axs[1, 0].set_xlabel('Time(s)')
            axs[1, 0].set_ylabel('Longitudinal velocity difference(m/s)')
            axs[1, 0].legend()
            
            # Plot lateral velocity difference
            axs[1, 1].plot(times, lat_vel_diff_l1_f1, 'r-', label='Lateral velocity difference between L1 and F1')
            axs[1, 1].plot(times, lat_vel_diff_f1_f2, 'b-', label='Lateral velocity difference between F1 and F2')
            axs[1, 1].set_title('(b) Lateral velocity difference')
            axs[1, 1].set_xlabel('Time(s)')
            axs[1, 1].set_ylabel('Lateral velocity difference(m/s)')
            axs[1, 1].legend()
            
            plt.tight_layout()
            plt.savefig('formation_plots.png', dpi=300)
            plt.savefig('formation_plots.pdf')
            self.get_logger().info("Static plots saved to formation_plots.png and formation_plots.pdf")
            
        except Exception as e:
            self.get_logger().error(f"Error generating plots from file: {e}")


def main(args=None):
    rclpy.init(args=args)
    data_collector = FormationDataCollector()
    
    try:
        rclpy.spin(data_collector)
    except KeyboardInterrupt:
        pass
    finally:
        # Save final data and generate plots before shutting down
        data_collector.save_data_to_file()
        data_collector.generate_plots_from_file()
        data_collector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

