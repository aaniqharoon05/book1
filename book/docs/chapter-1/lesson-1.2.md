---
sidebar_position: 2
---

# Lesson 1.2: Understanding ROS 2 Nodes: The Brain Cells

**Learning Objectives:**
*   Explain the concept of a ROS 2 node.
*   Understand the "one job, one node" philosophy.
*   Write a basic "Hello, World" node in Python using `rclpy`.

## One Job, One Node

If ROS 2 is the nervous system, a **node** is a single nerve cell or a small, specialized part of the brain. A node is just a program that performs a single task.

This is a core principle in ROS 2: each part of your robot's software should be a small, independent node.
*   A node for the camera.
*   A node for the left wheel motor.
*   A node for the right wheel motor.
*   A node for the AI that makes decisions.

Why? This makes your system incredibly robust and easy to manage. If the camera node crashes, the wheels can still be controlled. If you want to improve your navigation logic, you only need to work on the navigation node; you don't have to touch the camera or motor code.

**[DIAGRAM: A robot drawing is shown. Arrows point from its camera to a code box labeled "camera_node.py," from its wheels to a box labeled "motor_controller_node.py," and from its main computer chassis to a box labeled "navigator_node.py." This visually links physical parts to software nodes.]**

## Practical Example: A Simple Python Node

Let's write our very first node. We'll use the `rclpy` library, which is the official way to use ROS 2 with Python. This node won't do much—it will just start up and print a message.

```python
# First, we import the necessary libraries
import rclpy
from rclpy.node import Node

# We define our node as a Python class
class MyFirstNode(Node):
    # The constructor is called when we create an object from this class
    def __init__(self):
        # We call the parent class's constructor and give our node a name
        super().__init__('my_first_node')
        # We get the node's logger and print a message
        self.get_logger().info('Hello from my first ROS 2 node!')

def main(args=None):
    # Initialize the rclpy library
    rclpy.init(args=args)

    # Create an instance of our node
    node = MyFirstNode()

    # "Spin" the node, which keeps it running until it's shut down
    rclpy.spin(node)

    # Clean up and shutdown the node and rclpy
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

This code sets up all the essential parts of a ROS 2 program. The `main` function handles the startup and shutdown, and our `MyFirstNode` class is where our robot's logic will live.

## Recap:
*   A node is a program that performs a specific task.
*   Using many small nodes makes a robot's software easier to build and maintain.
*   We use the `rclpy` library to create nodes in Python.
*   Every node needs to be initialized, "spun" to keep it alive, and then shut down cleanly.

## Try It Yourself:
Save the code above as `my_node.py`. If you have ROS 2 installed, run it from your terminal. Then, try changing the node name from `'my_first_node'` to something else. Change the `info` message it prints. This simple exercise is the first step to building any robot program.
