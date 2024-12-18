
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get the path to the 'custom_robots' package
    pkgPath = get_package_share_directory('module_3_assignment')
    # Define the path to the RViz2 configuration file
    rvizConfigFilePath = os.path.join(pkgPath, 'config', 'urdf_view1.rviz')
    # Define the path to the URDF file
    urdfFile = os.path.join(pkgPath, 'urdf', 'arm_3dof_1.urdf')

    return LaunchDescription([
        # Node to publish the robot's URDF to the robot_state_publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdfFile]),
        # Node for interactive joint state publishing using a GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher',
            arguments=[urdfFile]),
        # Node to launch RViz2 with the specified configuration file
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rvizConfigFilePath]),
    ])
