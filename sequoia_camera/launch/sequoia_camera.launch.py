from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """Launch Sequoia camera component after unmounting GVFS."""
    
    imu_rate_arg = DeclareLaunchArgument(
        'imu_rate', default_value='1.0', description='IMU publishing rate in Hz'
    )
    sunshine_rate_arg = DeclareLaunchArgument(
        'sunshine_rate', default_value='1.0', description='Sunshine sensor publishing rate in Hz'
    )
    enable_imu_arg = DeclareLaunchArgument(
        'enable_imu', default_value='false', description='Enable IMU data publishing'
    )
    enable_sunshine_arg = DeclareLaunchArgument(
        'enable_sunshine', default_value='false', description='Enable sunshine sensor publishing'
    )

    container = ComposableNodeContainer(
        name='sequoia_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='sequoia_camera',
                plugin='sequoia_camera::SequoiaCameraComponent',
                name='sequoia_camera',
                parameters=[{
                    'imu_rate': LaunchConfiguration('imu_rate'),
                    'sunshine_rate': LaunchConfiguration('sunshine_rate'),
                    'enable_imu': LaunchConfiguration('enable_imu'),
                    'enable_sunshine': LaunchConfiguration('enable_sunshine'),
                }],
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        imu_rate_arg,
        sunshine_rate_arg,
        enable_imu_arg,
        enable_sunshine_arg,
        container,
    ])