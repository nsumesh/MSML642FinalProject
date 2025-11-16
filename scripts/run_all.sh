#!/bin/bash
set -e

# ======================================================
# Full TurtleBot3 Waffle Pi Simulation - Clean Baseline
# ======================================================

# Clean up any leftover ROS or Gazebo processes
echo "Cleaning up old processes..."
for p in ros2 gzserver gzclient teleop_twist_keyboard auto_explore; do
  pkill -9 $p 2>/dev/null || true
done
ros2 daemon stop || true
ros2 daemon start

# Source environment setup
SCRIPT_DIR=$(dirname "$0")
source $SCRIPT_DIR/setup_env.sh
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash

export TURTLEBOT3_MODEL=waffle_pi

#Set world path
WORLD_PATH="$SCRIPT_DIR/../gazebo_worlds/small_warehouse.world"

echo "======================================================"
echo " Starting Full TurtleBot3 Simulation (Waffle Pi)"
echo "======================================================"

#Launch Gazebo with the warehouse world
echo "Launching Gazebo..."
ros2 launch gazebo_ros gazebo.launch.py world:=$WORLD_PATH &
GAZEBO_PID=$!

# Wait until Gazebo master is ready
echo "Waiting for Gazebo master to initialize..."
until gz topic -l >/dev/null 2>&1; do
  sleep 2
done
sleep 3

# Spawn the TurtleBot3 Waffle Pi
echo "Spawning TurtleBot3 Waffle Pi..."
ros2 run gazebo_ros spawn_entity.py \
  -entity turtlebot3_waffle_pi \
  -file $(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models/turtlebot3_waffle_pi/model.sdf \
  -x 0 -y 0 -z 0.2 -Y 0 || true

echo "Waffle Pi successfully spawned!"

# Stop any leftover autonomous nodes and zero motion
echo "Ensuring robot stays still..."
pkill -9 auto_explore 2>/dev/null || true
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Launch RViz for visualization (optional)
echo "Launching RViz..."
ros2 launch nav2_bringup rviz_launch.py &
RVIZ_PID=$!

# Launch teleoperation in the same terminal (interactive)
echo "Starting keyboard teleoperation..."
echo "   Use 'i','j','k','l' to move. Press CTRL+C to stop."
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# Cleanup on exit
echo "Cleaning up Gazebo and RViz..."
kill $GAZEBO_PID 2>/dev/null || true
kill $RVIZ_PID 2>/dev/null || true

echo "======================================================"
echo "Simulation ended cleanly. All processes stopped."
echo "======================================================"
