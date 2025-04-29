ros2 service call /enable_motion std_srvs/srv/SetBool "data: true"
echo "Motion enabled"
ros2 topic pub /my_car/cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}' -1
echo "Obstacle Motion Enabled"
