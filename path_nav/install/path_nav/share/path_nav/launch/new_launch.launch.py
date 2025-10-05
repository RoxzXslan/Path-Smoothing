from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.event_handlers import OnProcessStart
from launch.actions import RegisterEventHandler
from launch.actions import TimerAction
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    os.environ['TURTLEBOT3_MODEL'] = 'burger'

    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py'
            )
        )
    )

    # RViz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', '/home/shamganesh/ros2_ws/src/path_nav/rviz/path_nav.rviz']
    )

    # Controller node
    controller_node = Node(
        package='path_nav',
        executable='pure_pursuit_controller',
        output='screen'
    )
    controller_with_delay = TimerAction(
        period=10.0, # small delay for rviz2 and gazebo to start smoothly, before controller node
        actions=[controller_node]
    )

    # Publisher node
    publisher_node = Node(
        package='path_nav',
        executable='trajectory_publisher',
        output='screen'
    )

    # publisher only goes in action after controller
    publisher_after_controller = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_node,
            on_start=[publisher_node]
        )
    )

    return LaunchDescription([
        turtlebot3_gazebo_launch,    
        rviz_node,
        controller_with_delay,
        publisher_after_controller
    ])
