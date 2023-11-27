from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
def generate_launch_description():
    publisher_freq_launch = DeclareLaunchArgument(
        "publish_frequency", default_value='500'
    )
    record_bag_arg = DeclareLaunchArgument(
        'record_bag',
        default_value='true',
        description='Enable/Disable bag recording'
    )
    record_bag_node = ExecuteProcess(
        condition=IfCondition(LaunchConfiguration('record_bag')),
        cmd=['ros2', 'bag', 'record', '-a'],
        output='screen'
    )


    return LaunchDescription([
        publisher_freq_launch,
        Node(
            package='beginner_tutorials', 
            executable='talker',
            parameters=[{"publish_frequency_ms": LaunchConfiguration('publish_frequency')}] # Pass the publish frequency in milliseconds
        ) ,record_bag_arg,record_bag_node

    ])
