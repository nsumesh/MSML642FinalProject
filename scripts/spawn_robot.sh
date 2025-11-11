#!/bin/bash
source $(dirname "$0")/setup_env.sh

echo "------------------------------------------------------"
echo "Spawning TurtleBot3 Waffle Pi in Gazebo..."
echo "------------------------------------------------------"

# Ensure the TurtleBot3 model is set correctly
export TURTLEBOT3_MODEL=waffle_pi

# Wait for Gazebo to initialize
sleep 3

# Spawn the Waffle Pi model from the ROS model database
ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3_waffle_pi \
  -database turtlebot3_waffle_pi \
  -x 0 -y 0 -z 0.01

echo "TurtleBot3 Waffle Pi spawned successfully!"


