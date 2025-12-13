---
sidebar_position: 3
---

# Lesson 1.3: Topics: The Robot's Message Bus

**Learning Objectives:**
*   Understand what a ROS 2 topic is and how it works.
*   Differentiate between a "publisher" and a "subscriber."
*   Write two nodes that communicate over a topic.

## Sending One-Way Messages

Our nodes are currently isolated. To be useful, they need to communicate. The most common way they do this is through **topics**.

A topic is like a channel on a radio. One node, called a **publisher**, sends messages on that channel. Any number of other nodes, called **subscribers**, can listen to that channel to receive the messages.

This is a one-way street. The publisher broadcasts its message to anyone who is listening. It doesn't know or care if anyone is actually tuned in. This "decoupled" communication is powerful because you can add or remove subscribers without ever changing the publisher.

For example, a camera node might *publish* images on a topic called `/camera/image`. A navigation node could *subscribe* to this topic to see obstacles, and a separate recording node could also *subscribe* to the same topic to save a video feed. The camera node doesn't need to change at all to support both.

**[DIAGRAM: A diagram with three boxes. The first box, labeled "Publisher Node (e.g., Camera)," has an arrow pointing to a central channel labeled "Topic: /camera_data". Two other boxes, "Subscriber Node 1 (e.g., Navigation)" and "Subscriber Node 2 (e.g., Recorder)," have arrows pointing from the topic to them, showing they both receive the same data.]**

## Practical Example: A Simple Publisher and Subscriber

Let's create two nodes. One will publish a simple "Hello" message with a counter, and the other will subscribe and print the message it receives.

First, we need to define the message type. For this, we'll use a standard string message.

**Publisher (`talker.py`):**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String # Standard message type for strings

class TalkerNode(Node):
    def __init__(self):
        super().__init__('talker_node')
        # Create a publisher on the 'chatter' topic, for String messages
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.timer_period = 0.5  # seconds
        # Create a timer that calls the timer_callback function every 0.5s
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker_node = TalkerNode()
    rclpy.spin(talker_node)
    talker_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Subscriber (`listener.py`):**
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ListenerNode(Node):
    def __init__(self):
        super().__init__('listener_node')
        # Create a subscriber on the 'chatter' topic, for String messages
        # When a message is received, the listener_callback is called
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        # Log the received message
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener_node = ListenerNode()
    rclpy.spin(listener_node)
    listener_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```
If you run both of these programs in separate terminals, you will see the `talker` publishing messages and the `listener` printing them. They are communicating via the `chatter` topic!

## Recap:
*   Topics are named channels for sending messages.
*   A **publisher** sends data to a topic.
*   A **subscriber** receives data from a topic.
*   This system is decoupled: publishers and subscribers don't need to know about each other directly.

## Try It Yourself:
Run the two nodes. Then, stop the `listener.py` node. What happens to the `talker.py` node? (It should keep running happily!) Now, restart the listener. Does it start receiving messages again? This demonstrates the power of decoupled communication.
