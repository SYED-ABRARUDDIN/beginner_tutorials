from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    publisher_freq_launch = DeclareLaunchArgument(
        "publish_frequency", default_value='500'
    )

    return LaunchDescription([
        publisher_freq_launch,
        Node(
            package='beginner_tutorials', 
            executable='talker',
            parameters=[{"publish_frequency_ms": LaunchConfiguration('publish_frequency')}] # Pass the publish frequency in milliseconds
        )  
    ])
