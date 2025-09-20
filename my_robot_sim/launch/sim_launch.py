import os
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    world_path = os.path.join(
        get_package_share_directory('my_robot_sim'),
        'worlds',
        'maze_lidar.sdf'
    )

    return LaunchDescription([
        # Start Gazebo with your world
        ExecuteProcess(
            cmd=['ign', 'gazebo', world_path],
            output='screen'
        ),
        # Start the bridge
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                 '/lidar@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
                 '/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist'],
            output='screen'
        ),
        # Start obstacle avoidance node
        Node(
            package='my_robot_sim',
            executable='obstacle_avoidance',
            output='screen'
        )
    ])
