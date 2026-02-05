---
sidebar_position: 1
---

# Chapter 1: ROS 2 Concepts: Nodes and Topics

Welcome to the first chapter of Module 1. In this chapter, we will explore the fundamental concepts of ROS 2: **Nodes** and **Topics**. These are the building blocks of any ROS 2 application, forming the "nervous system" of our robots.

## What is a ROS 2 Node?

A ROS 2 **Node** is the primary building block for creating a ROS 2 application. Think of a node as a small, single-purpose program that performs a specific task. For example, you might have one node for controlling a motor, another for reading sensor data, and a third for processing that data.

Each node in a ROS 2 system can communicate with other nodes, allowing you to create complex behaviors by combining simple, reusable components. This modular approach is one of the key strengths of ROS 2, as it makes your robotics software easier to develop, debug, and maintain.

## What is a ROS 2 Topic?

ROS 2 **Topics** are the communication channels that nodes use to exchange data. Nodes can **publish** messages to a topic or **subscribe** to a topic to receive messages. This publish/subscribe model allows for decoupled communication between nodes, meaning that a publishing node doesn't need to know which nodes are subscribing to its messages, and vice versa.

### Messages

Topics have a specific **message type**, which defines the structure of the data that is sent over the topic. ROS 2 provides a rich set of built-in message types for common robotics data, such as sensor readings, robot state, and control commands. You can also create your own custom message types to suit the specific needs of your application.

### How it Works

1.  A **publisher** node creates a message and publishes it to a specific topic.
2.  The ROS 2 middleware delivers the message to all **subscriber** nodes that are subscribed to that topic.
3.  Each subscriber node receives the message and can process the data as needed.

This one-to-many communication pattern is highly efficient and flexible, making it ideal for a wide range of robotics applications.

In the next section, we will see how to create our own ROS 2 nodes and use topics to send and receive data.

## Code Examples: Talker and Listener

Here are two simple Python examples demonstrating a "talker" node that publishes messages and a "listener" node that subscribes to them.

### Talker (Publisher)

This node publishes a "Hello World" message to the `/chatter` topic every second.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Talker(Node):
    def __init__(self):
        super().__init__('talker')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = String()
        msg.data = f'Hello World: {self.i}'
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1

def main(args=None):
    rclpy.init(args=args)
    talker = Talker()
    rclpy.spin(talker)
    talker.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Listener (Subscriber)

This node subscribes to the `/chatter` topic and prints the received messages to the console.

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Listener(Node):
    def __init__(self):
        super().__init__('listener')
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        self.get_logger().info(f'I heard: "{msg.data}"')

def main(args=None):
    rclpy.init(args=args)
    listener = Listener()
    rclpy.spin(listener)
    listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

