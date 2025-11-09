
echo "Setting up ROS2 + TurtleBot3 environment"
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export TURTLEBOT3_MODEL=waffle
export SDL_AUDIODRIVER=dummy
export GAZEBO_AUDIO=0
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:$(pwd)/gazebo_worlds

echo "Environment has been setup."

