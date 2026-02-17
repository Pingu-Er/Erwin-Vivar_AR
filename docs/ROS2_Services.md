# Activity 2: ROS 2 Services

## Overview

In this activity we extend a topic-based pipeline by adding a ROS 2 service that can reset the counter without stopping either node.

We need two Python nodes:

1. **Node 1: `number_publisher`**  
    - Publishes an **Int64** message on `/number` every 1 second.

2. **Node 2: `number_counter`** (subscriber + publisher + service server)  
    - Subscribes to **`/number` (Int64)**.
    - Accumulates the received value into an internal variable (the counter).
    - Republishes the accumulated value on **`/number_count` (Int64)** every 0.5 seconds.
    - Provides a service server `/reset_counter` of type example_interfaces/srv/SetBool to reset the accumulator to zero on demand.

Conceptually, it should show the following workflow:

![WIP](recursos/imgs/ROS2_srv/ROS2_service_ask.png)

---

## Service type

In this activity I used the server example_interfaces/srv/SetBool type. This service provides:

- Request: a boolean field (data)
- Response: a boolean flag (success) and a string (message)

You can inspect it directly with:

```bash
ros2 interface show example_interfaces/srv/SetBool
```

## Code

1. Publisher node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64

class MyPublisher(Node):
    def __init__(self):
        super().__init__("number_publisher")
        self.cnt = 0
        self.get_logger().info("number_publisher is alive")
        self.create_timer(1.0, self.talk)
        self.publisher_ = self.create_publisher(Int64, "number", 10)

    def talk(self):
        msg = Int64()
        msg.data = self.cnt
        self.cnt = 1
        self.get_logger().info("I said: ""Say: " + str(msg.data))
        self.publisher_.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)   
    number_publisher = MyPublisher()
    rclpy.spin(number_publisher) 
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

The code creates a publisher (`number_publisher`) that sends integer messages on the `/number` topic using `example_interfaces/Int64`. A timer calls `talk()` once per second, where an `Int64` message is created, its `data` field is filled with the current value of `self.cnt`, the value is logged, and then published. In this implementation, after the first publish the variable is forced to `1` (`self.cnt = 1`), so the node effectively outputs a constant increment tick (0 once, then 1 repeatedly). The `main()` function initializes ROS 2, instantiates the node, keeps it running with `rclpy.spin()`, and shuts down cleanly.


2. Publisher, subscriber and server node

```python
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class MyPublisher(Node):
    def __init__(self):
        super().__init__("number_counter")
        self.cntP = 0
        self.get_logger().info("number_counter is alive")
        self.create_timer(0.5, self.talk)
        
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.subscriber = self.create_subscription(Int64,"number", self.listen, 10)
        self.server = self.create_service(SetBool,"reset_counter", self.call_reset) #Service type, Service name, callback

    def call_reset(self, request: SetBool.Request, response: SetBool.Response):
        if (request.data):
            self.cntP = 0
            request.data = False
            response.message = "Reset"
        else:
            response.message = "War never changes"
    
        return response

    def listen(self, msg: Int64):
        self.get_logger().info(str(msg.data))
        self.cntP = self.cntP + msg.data

    def talk(self):
        msg = Int64()
        msg.data = self.cntP
        self.get_logger().info("number_publisher is saying: " + str(msg.data))
        self.publisher_.publish(msg)
            

def main(args=None):
    rclpy.init(args=args)   
    number_counter = MyPublisher()
    rclpy.spin(number_counter) 
    rclpy.shutdown()

if __name__ == "__main__":
    main()
```

The second code creates the `number_counter` node, which combines a subscriber, a publisher, and a service server. It subscribes to `/number` (`Int64`) and, every time a message arrives, the `listen()` callback logs the incoming integer and accumulates it into `self.cntP` (`self.cntP = self.cntP + msg.data`). In parallel, a timer calls `talk()` every 0.5 seconds to publish the current accumulated value on `/number_count` using `example_interfaces/Int64`.

```python
    def call_reset(self, request: SetBool.Request, response: SetBool.Response):
        if (request.data):
            self.cntP = 0
            request.data = False
            response.message = "Reset"
        else:
            response.message = "War never changes"
    
        return response
```

Additionally, the node exposes the `/reset_counter` service with type `example_interfaces/srv/SetBool`. When the service is called, the callback `call_reset()` checks `request.data`: if it is `true`, the node resets the accumulator (`self.cntP = 0`) and returns a response message `"Reset"`; otherwise it leaves the counter unchanged and responds with `"War never changes"`. As before, `main()` initializes ROS 2, spins the node to keep callbacks active (topics + service), and shuts down on exit.

---

## Terminal Commands


1. Run the nodes
    - number_publisher node
```bash
ros2 run er_pkg p2d2 
```

    - number_counter node
```bash
ros2 run er_pkg h1
```

2. Calling the Reset Service from the CLI
To request a reset (send data: true):
```bash
ros2 service call /reset_counter example_interfaces/srv/SetBool "{data: true}"
```

3. Verifying the System Graph
```bash
rqt_graph
```

---
## Results (Terminal and rqt_graph)

![WIP](recursos/imgs/ROS2_srv/ROS2_service_result.png)
