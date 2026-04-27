from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, LoadComposableNodes
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        prefix = 'xterm -e',
        output='screen',
    )
    load_nodes = LoadComposableNodes(
        target_container='container',
        composable_node_descriptions=[
            ComposableNode(
                package='bme_gazebo_sensors',
                plugin='GoToServer',
                name='goto_server',
            ),
            ComposableNode(
                package='bme_gazebo_sensors',
                plugin='GoToClient',
                name='goto_client',
            ),
        ],
    )

    return LaunchDescription([
        container,
        load_nodes
    ]) 