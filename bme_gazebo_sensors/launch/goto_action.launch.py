from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
            name='my_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='bme_gazebo_sensors',
                    plugin='GoToServer',
                    name='server',
                )
            ],
            output='screen',
        )
            # Avvia il Client
    node = Node(
        package='bme_gazebo_sensors',
        executable='goto_client',
        name='goto_client',
        output='screen'
    )

    return LaunchDescription([
        container,
        node
    ])
