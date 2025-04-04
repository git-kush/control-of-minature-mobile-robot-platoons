import numpy as np
import time

class OvertakeController:
    def __init__(self):
        # Safety parameters
        self.safe_distance = 0.6  # meters
        self.speed_threshold = 0.2  # m/s
        self.overtake_timeout = 25  # seconds
        self.cooldown_duration = 10  # seconds

        # State variables
        self.overtake_flag = False
        self.abort_flag = False
        self.last_min_distance = None
        self.obstacle_velocity = 0.0
        self.virtual_node_position = None
        self.obstacle_position = None
        self.overtake_start_time = 0
        self.last_overtake_end = 0
        self.smoothing_alpha = 0.6

        # Virtual node parameters
        self.virtual_node_offset = 1.0  # meters ahead

    def update(self, distances, goal_position, leader_position, dt=0.05):
        """Update overtaking logic at 20Hz (dt=0.05s)"""
        current_time = time.time()

        # Step 1: Find minimum distance
        min_distance = np.min(distances)
        min_index = np.argmin(distances)

        # Step 2: Check safe distance
        if min_distance < self.safe_distance:
            # Step 3: Calculate obstacle velocity
            if self.last_min_distance is not None:
                raw_velocity = (self.last_min_distance - min_distance) * 20  # 20Hz sampling
                # Exponential smoothing
                self.obstacle_velocity = (self.smoothing_alpha * raw_velocity +
                                         (1 - self.smoothing_alpha) * self.obstacle_velocity)

            # Step 4: Check speed threshold and cooldown
            if (self.obstacle_velocity > self.speed_threshold and
                (current_time - self.last_overtake_end) > self.cooldown_duration):
                if not self.overtake_flag:
                    # Initialize overtake
                    self.overtake_flag = True
                    self.overtake_start_time = current_time
                    # Step 5: Create virtual node
                    self.virtual_node_position = leader_position + np.array([self.virtual_node_offset, 0])
                    self.obstacle_position = leader_position + np.array([min_distance, 0])  # Simplified

            self.last_min_distance = min_distance
        else:
            self.last_min_distance = None
            self.obstacle_velocity = 0.0

        # Update virtual node position
        if self.overtake_flag and self.obstacle_position is not None:
            # Step 6: Move virtual node with obstacle
            self.obstacle_position += self.obstacle_velocity * dt * np.array([1, 0])  # Assuming X-axis motion
            self.virtual_node_position = self.obstacle_position + np.array([self.virtual_node_offset, 0])

        # Step 7: Check overtake completion or timeout
        if self.overtake_flag:
            # Check goal proximity (use your existing goal check)
            goal_distance = np.linalg.norm(goal_position - leader_position)
            if goal_distance < 0.1:  # Your existing threshold
                self._end_overtake()
            elif (current_time - self.overtake_start_time) > self.overtake_timeout:
                self._abort_overtake()

        return self.overtake_flag, self.virtual_node_position

    def _end_overtake(self):
        self.overtake_flag = False
        self.last_overtake_end = time.time()
        self.virtual_node_position = None

    def _abort_overtake(self):
        self.abort_flag = True
        self._end_overtake()
        print("Overtake aborted due to timeout")

    # Integrate with your existing APF code like this:
    def calculate_attractive_force(self, leader_pos, goal_pos, virtual_node_pos=None):
        """Modified attractive force calculation"""
        if self.overtake_flag and virtual_node_pos is not None:
            # Calculate vector to virtual node instead of goal
            displacement = virtual_node_pos - leader_pos
        else:
            displacement = goal_pos - leader_pos

        # Your existing code for resolving into x/y components
        force_magnitude = np.linalg.norm(displacement)
        longitudinal_force = displacement[0] / force_magnitude
        lateral_force = displacement[1] / force_magnitude

        return longitudinal_force, lateral_force
