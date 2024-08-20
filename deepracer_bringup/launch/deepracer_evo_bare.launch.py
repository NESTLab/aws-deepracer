from launch import LaunchDescription
import launch_ros.actions

def generate_launch_description():

    return LaunchDescription([
        launch_ros.actions.Node(
            package='camera_pkg',
            namespace='camera_pkg',
            executable='camera_node',
            name='camera_node',
            parameters=[
                {'resize_images': False}
            ]
        ),

        launch_ros.actions.Node(
            package='rplidar_ros',
            namespace='',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                    'serial_port': '/dev/ttyUSB0',
                    'serial_baudrate': 115200,
                    'frame_id': 'laser_frame',
                    'inverted': False,
                    'angle_compensate': True,
            }]
        ),

        launch_ros.actions.Node(
            package='servo_pkg',
            namespace='servo_pkg',
            executable='servo_node',
            name='servo_node',
            remappings=[('/ctrl_pkg/servo_msg', '/cmdvel_to_servo_pkg/servo_msg')]
        ),

        launch_ros.actions.Node(
            package='cmdvel_to_servo_pkg',
            namespace='cmdvel_to_servo_pkg',
            executable='cmdvel_to_servo_node',
            name='cmdvel_to_servo_node'
        ),

        launch_ros.actions.Node(
            package='imu_pkg',
            namespace='imu_pkg',
            executable='imu_node',
            name='imu_node'
        )
    ])