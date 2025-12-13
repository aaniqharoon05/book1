--- 
sidebar_position: 4
---

# Lesson 1.4: Services & Actions: Two-Way Conversations

**Learning Objectives:**
*   Describe when to use a service instead of a topic.
*   Explain the difference between a service (quick request) and an action (long task).
*   Understand the structure of a ROS 2 service (client and server).

## Beyond One-Way Shouting

Topics are great for continuous streams of data, like a camera feed. But what if you need to ask a question and get a direct answer? Or what if you need to command a robot to perform a long task and get updates along the way?

For this, we have **Services** and **Actions**.

### Services: A Quick Question and Answer

A service is a **two-way** conversation. A node, called the **client**, sends a request to another node, called the **server**. The server does some work and sends a single response back to the client.

This is a blocking call, like ordering coffee. You (the client) ask the barista (the server) for a latte. You wait right there until you get your latte (the response). You wouldn't use this for a long task, just as you wouldn't stand at the counter for 20 minutes waiting for a complicated meal.

*   **Use a service for:** Quick, remote procedure calls.
    *   Example: A node asks a `calculator_server` to add two numbers. The server responds with the sum.
    *   Example: A navigation node asks a `map_server` for the current map data.

**[DIAGRAM: A "Client Node" box with an arrow labeled "Request (e.g., Add 2+2)" pointing to a "Service Server Node" box. A second arrow points back from the server to the client, labeled "Response (e.g., 4)".]**

### Actions: Long-Running Tasks with Feedback

An **action** is for tasks that take a long time. Think of ordering a pizza.
1.  You (the **action client**) call the pizza place and give them a goal (a large pepperoni pizza).
2.  The pizza place (the **action server**) accepts the goal.
3.  While they're making it, they might give you **feedback** ("The pizza is in the oven").
4.  When it's done, they give you a final **result** ("Your pizza is ready").
You can also cancel the order at any time.

Actions are perfect for tasks like "navigate to the kitchen," "pick up the red ball," or "scan the room for 30 seconds."

## Practical Example: A Simple Service

Let's create a service that adds two numbers.

First, we'd define a `.srv` file:
```
int64 a
int64 b
---
int64 sum
```
This defines the `Request` (two integers, a and b) and the `Response` (one integer, sum).

Here is the Python code for the server.

**Service Server (`add_two_ints_server.py`):**
```python
# We would import the service type we defined, e.g., from example_interfaces.srv import AddTwoInts
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts # A pre-built service for this!

class MinimalService(Node):
    def __init__(self):
        super().__init__('minimal_service')
        # Create the service, linking it to our callback function
        self.srv = self.create_service(AddTwoInts, 'add_two_ints', self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        # The request object holds the incoming data
        response.sum = request.a + request.b
        # Log the request and the response we're sending
        self.get_logger().info(f'Incoming request\na: {request.a} b: {request.b}')
        self.get_logger().info(f'Sending back response: [{response.sum}]
')
        # Return the populated response object
        return response

def main(args=None):
    rclpy.init(args=args)
    minimal_service = MinimalService()
    rclpy.spin(minimal_service)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
The service client would then call this service, send `a` and `b`, and wait for the `sum` to come back.

## Recap:
*   **Topics** are for continuous, one-way data streams.
*   **Services** are for quick, two-way request/response interactions.
*   **Actions** are for long-running tasks that require feedback and can be cancelled.
*   Choosing the right tool (Topic, Service, or Action) is a key part of designing a good robotics system.

## Try It Yourself:
Think about our robot vacuum again. Which communication type would you use for the following?
1.  Continuously sending out laser scan data.
2.  Asking the battery node "What is your current charge level?"
3.  Telling the navigation system "Go charge yourself," a task that could take minutes.
*(Answers: 1. Topic, 2. Service, 3. Action)*
