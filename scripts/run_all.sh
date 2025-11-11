#!/bin/bash
set -e

# Source environment setup
source $(dirname "$0")/setup_env.sh

echo "======================================================"
echo " Starting Full TurtleBot3 Simulation (Waffle Pi)"
echo "======================================================"

export TURTLEBOT3_MODEL=waffle_pi

WORLD_PATH="$(dirname "$0")/../gazebo_worlds/small_warehouse.world"

# Start Gazebo with ROS factory plugin
echo "Launching Gazebo + ROS integration..."
ros2 launch gazebo_ros gazebo.launch.py world:=$WORLD_PATH &
GAZEBO_PID=$!

sleep 10

# Spawn the TurtleBot3 Waffle Pi
echo "Spawning TurtleBot3 Waffle Pi..."
ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3_waffle_pi \
  -file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x 0 -y 0 -z 0.01

echo "Waffle Pi successfully spawned!"

# Launch teleop
echo "Starting keyboard teleoperation..."
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Cleanup
kill $GAZEBO_PID 2>/dev/null || true

