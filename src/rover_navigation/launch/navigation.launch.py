from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rover/pose@geometry_msgs/msg/Pose@gz.msgs.Pose',
            '/rover/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
            '/model/rover/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/world/rover_map/create@ros_gz_interfaces/srv/SpawnEntity',
            '/world/rover_map/remove@ros_gz_interfaces/srv/DeleteEntity',
        ],
        output='screen',
    )

    navigator = Node(
        package='rover_navigation',
        executable='simple_navigator',
        output='screen',
    )

    return LaunchDescription([bridge, navigator])
