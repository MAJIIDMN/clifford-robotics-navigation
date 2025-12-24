import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    package_share = get_package_share_directory('robot_ga')
    rviz_config = os.path.join(package_share, 'rviz', 'config.rviz')
    
    # Set environment variables untuk Wayland/X11 compatibility
    set_display = SetEnvironmentVariable('DISPLAY', ':0')
    unset_wayland = SetEnvironmentVariable('WAYLAND_DISPLAY', '')
    set_xcursor = SetEnvironmentVariable('QT_QPA_PLATFORM', 'xcb')
    
    return LaunchDescription([
        set_display,
        unset_wayland,
        set_xcursor,
        Node(
            package='robot_ga',
            executable='motion_node',
            name='ga_motion_node',
            output='screen'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],
            output='screen'
        )
    ])
