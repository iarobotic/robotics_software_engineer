# Launches a robot driver node for controlling the TurtleBot3 in a ROS 2 environment.
# - Utilizes the `drive_mobile_robot` package for robot control logic.
# - Configures the robot driver node to subscribe to `/cmd_vel` for velocity commands.
# Author: Robotisim

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    # Define the Turtlesim node
    turtlesim = Node(
        package='turtlesim',  # Name of the package where the turtlesim_node is located
        executable='turtlesim_node',  # Name of the executable to run
        name='turtlesim'  # Name assigned to the node
    )

    patter_node=Node(
        package='module_2_assignment',
        executable='pattern',
        name='pattern',
        parameters=[
            {'radius': 0.9},
            {'pattern': 'spiral'},
        ]
    )

    return LaunchDescription([
        turtlesim,
        patter_node,

    ])