from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    world = '/home/nsumesh/MSML_642_FinalProject/gazebo_worlds/small_warehouse.world'

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([get_package_share_directory('gazebo_ros'),
                                  'launch', 'gazebo.launch.py'])
        ),
        launch_arguments={'world': world}.items()
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_tb3',
        arguments=[
            '-file', PathJoinSubstitution([
                get_package_share_directory('turtlebot3_gazebo'),
                'models', 'turtlebot3_waffle_pi', 'model.sdf'
            ]),
            '-entity', 'tb3', '-x', '0.0', '-y', '0.0', '-z', '0.1'
        ],
        output='screen'
    )

    env = Node(
        package='rl_nav',
        executable='tb3_env',
        name='tb3_env',
        output='screen'
    )

    return LaunchDescription([gazebo, spawn, env])
