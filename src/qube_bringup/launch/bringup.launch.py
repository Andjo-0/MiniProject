#Modified launch file from https://github.com/ros/urdf_launch/blob/main/launch/display.launch.py
#This has been modified by ChatGPT to not rely on the urdf_launch file from the ros2 urdf_tutorial

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Declare launch arguments
    urdf_package = LaunchConfiguration('urdf_package')
    urdf_path = LaunchConfiguration('urdf_package_path')
    baud_rate = LaunchConfiguration('baud_rate')
    device = LaunchConfiguration('device')
    simulation = LaunchConfiguration('simulation')
    rviz_config = LaunchConfiguration('rviz_config')

    # Find xacro file to process
    xacro_file = PathJoinSubstitution([
        FindPackageShare(urdf_package),
        urdf_path
    ])

    # Convert xacro to robot_description
    xacro_cmd = Command([
        'xacro ', xacro_file,
        ' baud_rate:=', baud_rate,
        ' device:=', device,
        ' simulation:=', simulation
    ])

    robot_description = {
        'robot_description': ParameterValue(xacro_cmd, value_type=str)
    }

    # Path to RViz config
    default_rviz_config_path = PathJoinSubstitution([
        FindPackageShare('urdf_launch'),
        'config',
        'urdf.rviz'
    ])

    # Path to qube_driver launch file
    qube_driver_launch = PathJoinSubstitution([
        FindPackageShare("qube_driver"),
        "launch",
        "qube_driver.launch.py"
    ])

    return LaunchDescription([
        # Declare launch args
        DeclareLaunchArgument('urdf_package', default_value='qube_bringup'),
        DeclareLaunchArgument('urdf_package_path', default_value='urdf/controlled_qube.urdf.xacro'),
        DeclareLaunchArgument('baud_rate', default_value='115200'),
        DeclareLaunchArgument('device', default_value='/dev/ttyACM0'),
        DeclareLaunchArgument('simulation', default_value='true'),
        DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path),

        # Publish the robot description to TF
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[robot_description],
            output='screen'
        ),

        # Initialises the rviz2 node
        Node(
            package='rviz2',
            executable='rviz2',
            output='screen',
            arguments=['-d', rviz_config],
        ),

        # Launch qube_driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(qube_driver_launch)
        )
    ])
