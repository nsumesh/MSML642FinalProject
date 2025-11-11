#!/bin/bash
source $(dirname "$0")/setup_env.sh
echo "Starting keyboard teleop. Use 'i' to move forward, 'k' to stop."
ros2 run teleop_twist_keyboard teleop_twist_keyboard

