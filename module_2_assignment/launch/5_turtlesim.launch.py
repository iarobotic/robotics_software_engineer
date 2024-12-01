# Launches a Turtlesim environment, spawns a second turtle, and controls both turtles.
# - Uses the turtlesim package to launch a basic turtle simulator.
# - Spawns a second turtle in the simulator.
# - Drives the original and the second turtle with separate nodes.

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    # Node configuration for launching the Turtlesim node
    turtlesimNode = Node(
        package='turtlesim',
        executable='turtlesim_node',
        name='turtlesim',
    )

    # ExecuteProcess used to spawn a second turtle in the Turtlesim environment
    spawnTurtle2 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 0.0, y: 11.0, theta: 0.0, name: 'turtle2'}\""],
        name='spawn_turtle2',
        shell=True
    )


    # ExecuteProcess used to spawn a second turtle in the Turtlesim environment
    spawnTurtle3 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 8.25, y: 2.75, theta: 0.0, name: 'turtle3'}\""],
        name='spawn_turtle3',
        shell=True
    )

    # ExecuteProcess used to spawn a second turtle in the Turtlesim environment
    spawnTurtle4 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 2.25, y: 8.25, theta: 0.0, name: 'turtle4'}\""],
        name='spawn_turtle4',
        shell=True
    )

    # ExecuteProcess used to spawn a second turtle in the Turtlesim environment
    spawnTurtle5 = ExecuteProcess(
        cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', "\"{x: 11.0, y: 0.0, theta: 0.0, name: 'turtle5'}\""],
        name='spawn_turtle5',
        shell=True
    )

    # Node configuration for driving the second spawned turtle
    turtMove3 = Node(
        package='module_2_assignment',
        executable='move_turtle',
        name='move_turtle3',
        parameters=[{'cmd_vel_topic': '/turtle3/cmd_vel'}]
    )
    
    # Node configuration for driving the second spawned turtle
    turtMove1 = Node(
        package='module_2_assignment',
        executable='move_turtle',
        name='move_turtle1',
    )
    
        # Node configuration for driving the second spawned turtle
    turtMove4 = Node(
        package='module_2_assignment',
        executable='move_turtle',
        name='move_turtle4',
        parameters=[{'cmd_vel_topic': '/turtle4/cmd_vel'}]
    )
    return LaunchDescription([
        turtlesimNode,
        spawnTurtle2,
        spawnTurtle3,
        spawnTurtle4,
        spawnTurtle5,
        turtMove3,
        turtMove1,
        turtMove4
    ])
