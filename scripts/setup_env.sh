#!/bin/bash
echo "------------------------------------------------------"
echo "Setting up ROS2 + TurtleBot3 + Gazebo environment..."
echo "------------------------------------------------------"

# --- ROS2 setup ---
if [ -f /opt/ros/humble/setup.bash ]; then
    source /opt/ros/humble/setup.bash
else
    echo "ROS2 Humble not found. Please install ROS2 Humble first."
    exit 1
fi

export TURTLEBOT3_MODEL=burger

if [ -f /usr/local/share/gazebo-11/setup.sh ]; then
    source /usr/local/share/gazebo-11/setup.sh
elif [ -f /usr/local/share/gazebo/setup.sh ]; then
    source /usr/local/share/gazebo/setup.sh
elif [ -f /usr/share/gazebo-11/setup.sh ]; then
    source /usr/share/gazebo-11/setup.sh
elif [ -f /usr/lib/x86_64-linux-gnu/gazebo-11/setup.sh ]; then
    source /usr/lib/x86_64-linux-gnu/gazebo-11/setup.sh
elif [ -f /usr/lib/aarch64-linux-gnu/gazebo-11/setup.sh ]; then
    source /usr/lib/aarch64-linux-gnu/gazebo-11/setup.sh
elif [ -f /usr/share/gazebo/setup.sh ]; then
    source /usr/share/gazebo/setup.sh
else
    echo "Gazebo setup.sh not found — continuing anyway."
fi

# --- Add project model and resource paths ---
PROJECT_DIR=$(dirname "$(dirname "$0")")
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$PROJECT_DIR/models:$PROJECT_DIR/models/aws_small_warehouse
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/usr/share/gazebo-11:/usr/local/share/gazebo-11

# --- Summary output ---
echo "Environment configured."
echo "TURTLEBOT3_MODEL: $TURTLEBOT3_MODEL"
echo "GAZEBO_MODEL_PATH: $GAZEBO_MODEL_PATH"
echo "GAZEBO_RESOURCE_PATH: $GAZEBO_RESOURCE_PATH"
echo "------------------------------------------------------"

