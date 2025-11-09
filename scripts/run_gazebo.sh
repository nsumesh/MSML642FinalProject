bash scripts/setup_env.sh
echo "Launching Gazebo with small warehouse world"
ros2 launch gazebo_ros gazebo.launch.py \
  world:=$HOME/aws-robomaker-small-warehouse-world/worlds/small_warehouse.world
