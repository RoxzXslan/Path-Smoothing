from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():
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
        rviz_node,
        controller_node,
        publisher_after_controller
    ])
