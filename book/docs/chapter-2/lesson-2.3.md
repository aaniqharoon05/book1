---
sidebar_position: 3
---

# Lesson 2.3: Spawning Your Robot in Gazebo

**Learning Objectives:**
*   Understand how to include a URDF robot model in a Gazebo world.
*   Learn about the `gazebo_ros_pkgs` for connecting ROS 2 to Gazebo.
*   Use a ROS 2 launch file to start Gazebo and spawn a robot.

## Bringing Your Robot to Life

We have a world, and from Chapter 1, we have a URDF file that describes our robot's body. Now, it's time to put them together.

You can't just put a URDF file directly into a Gazebo `.world` file. URDF is a general robot description, but Gazebo needs extra information, like colors and physics properties (e.g., friction). We can add special `<gazebo>` tags to our URDF to provide this information.

The most common way to get a robot into Gazebo, however, is to use a ROS 2 **launch file**. A launch file is a powerful script that can start multiple nodes and programs at once. We can write a launch file that does three things:
1.  Starts the Gazebo simulator and loads our world file.
2.  Finds our robot's URDF file.
3.  Runs a special `spawn_entity.py` node that takes the URDF and officially "spawns" it into the running simulation.

This connection between ROS 2 and Gazebo is handled by a package called `gazebo_ros_pkgs`. It provides the bridge that lets our ROS 2 nodes talk to the Gazebo simulation.

**[DIAGRAM: A flow chart. Box 1: "ROS 2 Launch File (.launch.py)" has an arrow to Box 2: "Starts Gazebo + my_world.world". A second arrow from Box 1 points to Box 3: "Loads robot.urdf". A third arrow from Box 1 points to Box 4: "Runs 'spawn' Node". An arrow from Box 3 and Box 4 points to Box 2, showing the robot description being injected into the simulation.]**

## Practical Example: A Launch File to Spawn a Robot

This is a typical launch file for spawning a URDF-defined robot into Gazebo. It's complex, but it's a very common pattern you will use often.

```python
# A simplified launch file to start Gazebo and spawn a robot
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Path to the Gazebo ROS package
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    # Path to your custom robot package
    pkg_my_robot = get_package_share_directory('my_robot_description')

    # Get the path to your world file
    world_file_path = os.path.join(pkg_my_robot, 'worlds', 'my_world.world')
    # Get the path to your robot's URDF file
    urdf_file_path = os.path.join(pkg_my_robot, 'urdf', 'simple_bot.urdf')

    # --- Actions to Launch ---

    # 1. Start Gazebo with your world
    start_gazebo_server_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file_path}.items()
    )

    start_gazebo_client_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # 2. Spawn the robot from the URDF file
    spawn_entity_cmd = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'simple_bot',
            '-file', urdf_file_path,
            '-x', '0',
            '-y', '0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        start_gazebo_server_cmd,
        start_gazebo_client_cmd,
        spawn_entity_cmd,
    ])
```
This script automates the entire process. When you run it, Gazebo will open, your world will load, and your robot will appear at the specified coordinates.

## Recap:
*   You need to add Gazebo-specific tags (`<gazebo>`) to your URDF or use plugins to make it work in the simulation.
*   The `gazebo_ros_pkgs` package provides the connection between ROS 2 and Gazebo.
*   A ROS 2 **launch file** is the standard way to start Gazebo and spawn your robot model into the world.

## Try It Yourself:
Look at the `spawn_entity_cmd` in the launch file. Can you see the arguments that define the robot's name (`-entity`) and its starting position (`-x`, `-y`, `-z`)? Try changing the x and y values to see if you can predict where the robot will appear in the world.
