#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    ld = LaunchDescription()

    TURTLEBOT3_MODEL = "burger"

    enable_drive = LaunchConfiguration("enable_drive", default="true")
    declare_enable_drive = DeclareLaunchArgument(
        name="enable_drive", default_value="true", description="Enable robot drive node"
    )

    turtlebot3_multi_robot = get_package_share_directory("turtlebot3_multi_robot")
    world = os.path.join(turtlebot3_multi_robot, "worlds", "road_track_small.world")

    urdf_file_name = "turtlebot3_" + TURTLEBOT3_MODEL + ".urdf"
    urdf = os.path.join(turtlebot3_multi_robot, "urdf", urdf_file_name)

    # Start Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzserver.launch.py")
        ),
        launch_arguments={"world": world}.items(),
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("gazebo_ros"), "launch", "gzclient.launch.py")
        ),
    )

    ld.add_action(declare_enable_drive)
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)

    # Define intervehicular distance and number of robots
    INTERVEHICULAR_DISTANCE = 1.0  # Meters
    NUM_BOTS = 3

    x_position = 9.0

    for i in range(NUM_BOTS):
        name = f"turtlebot{i}"
        namespace = f"/tb{i}"

        # State publisher for each robot
        turtlebot_state_publisher = Node(
            package="robot_state_publisher",
            namespace=namespace,
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"use_sim_time": False, "publish_frequency": 10.0}],
            arguments=[urdf],
        )

        # Spawn the TurtleBot3 in Gazebo
        spawn_turtlebot3_burger = Node(
            package="gazebo_ros",
            executable="spawn_entity.py",
            arguments=[
                "-file",
                os.path.join(turtlebot3_multi_robot, 'models', 'turtlebot3_' + TURTLEBOT3_MODEL, 'model.sdf'),
                "-entity", name,
                "-robot_namespace", namespace,
                "-x", str(x_position),
                "-y", "0.0",
                "-z", "0.01",
                "-Y", "3.14159",
                "-unpause",
            ],
            output="screen",
        )

        # Move the next robot to the right
        x_position += INTERVEHICULAR_DISTANCE

        # Add robots to launch description
        ld.add_action(turtlebot_state_publisher)
        ld.add_action(spawn_turtlebot3_burger)

    # Start teleop node for leader bot (tb0)
    # teleop_turtlebot3 = Node(
    #     package="turtlebot3_teleop",
    #     executable="teleop_keyboard",
    #     name="teleop_turtlebot3",
    #     output="screen",
    #     remappings=[("/cmd_vel", "/tb0/cmd_vel")]
    # )
    # ld.add_action(teleop_turtlebot3)


    lane_changing_node = Node(
        package="turtlebot3_lane_changing_leader",
        executable="lane_changing_leader",
        name="apf_controller",
        parameters=[
            {
            'initial_speed': 0.4,
            'goal_x': -9.0,
            'goal_y': 0.0,
            'goal_threshold': 0.5
            }  
        ],
        output="screen"
    )
    ld.add_action(lane_changing_node)




    # Leader-Follower Control Nodes for tb1 and tb2
    for i in range(1, NUM_BOTS):  # Followers only (tb1, tb2)
        leader_odom = "/tb0/odom" if i == 1 else f"/tb{i-1}/odom"  # tb1 follows tb0, tb2 follows tb1
        leader_follower_control = Node(
            package="turtlebot3_follower_control",
            executable="follower_control",
            name=f"follower_control_tb{i}",
            namespace=f"/tb{i}",
            output="screen",
            parameters=[{
                "leader_odom_topic": leader_odom,
                "follower_odom_topic": f"/tb{i}/odom",
                "follower_cmd_vel_topic": f"/tb{i}/cmd_vel",
                "desired_distance": INTERVEHICULAR_DISTANCE
            }]
        )
        ld.add_action(leader_follower_control)

    return ld
