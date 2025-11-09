bash scripts/setup_env.sh
echo "TurtleBot3 Waffle"

TB3_GZ_DIR=$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo
ros2 run gazebo_ros spawn_entity.py \
  -entity tb3_gz \
  -file $TB3_GZ_DIR/models/turtlebot3_waffle/model.sdf \
  -x 0 -y 0 -z 0.3 -Y 0
