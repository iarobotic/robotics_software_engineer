 # Module 2 Assignment: Developing Custom ROS 2 Nodes and Launch Files

## How to tun this

 ros2 launch module_2_assignment pattern.launch.py
 ros2 launch module_2_assignment 5_turtlesim.launch.py
 ros2 param set /move_turtle1 velocity 4.0


## My understanding Tasks

### Task 1: Create a Custom ROS 2 Node

- **Develop a ROS 2 node** 
  - **Circle Movement:** 
  - **Logarithmic Spiral Movement:**

  I create a patter_node node that allows generating a circle or spiral pattern that can be selected using a parameter. With another additional parameter for the radius.

### Task 2: Develop a Launch File

- **Create a launch file** 

- **Ensure proper documentation** 

### Task 3: Modify the Turtlesim Simulation Environment

- **Use existing Turtlesim services**
  - **Spawn 5 Turtlebots** 
  I create a launch file where the turtlesim are generated in certain positions with the /spawn' service.

  - **Drive the middle 3 turtles** 
  I create a move_turtle node that allows me to move the turtlesim back and forth. It has a parameter for the '/cmd_vel' topic so that velocities can be published independently.

### Task 4: Modify Turtle Behavior with Parameters

- **Utilize ROS 2 parameters** 
  - **Change the speed** 
  
I create a velocity parameter that I assign to linear.x. I change this parameter dynamically with the command 'ros2 param set'.


