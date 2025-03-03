# ROS2 node

## Create node process
1. Programming interface initialization
2. Create nodes and initialize them
3. Implement node functions
4. Destroy nodes and close interfaces

## Hello World node case

### Create python function package

```
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src
ros2 pkg create pkg_helloworld_py --build-type ament_python --dependencies rclpy --node-name helloworld
```

### Writing code

After executing the above command, ```pkg_helloworld_py``` will be created, and the ```helloworld.py``` file will be created to write the node:

```
import rclpy                                     # ROS2 Python interface library
from rclpy.node import Node                      # ROS2 Node class
import time

"""
Create a HelloWorld node and output the "hello world" log during initialization
"""
class HelloWorldNode(Node):
    def __init__(self, name):
        super().__init__(name)                     # ROS2 node parent class initialization
        while rclpy.ok():                          # Whether the ROS2 system is running normally
            self.get_logger().info("Hello World")  # ROS2 log output
            time.sleep(0.5)                        # sleep control loop time

def main(args=None):                               # ROS2 node main entrance main function
    rclpy.init(args=args)                          # ROS2 Python interface initialization
    node = HelloWorldNode("helloworld")            # Create a ROS2 node object and initialize it
    rclpy.spin(node)                               # Loop waiting for ROS2 to exit
    node.destroy_node()                            # Destroy node object
    rclpy.shutdown()                               # Close the ROS2 Python interface
```

### Compile function package

```
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select pkg_helloworld_py
source install/setup.bash
```

### Run the node

```
ros2 run pkg_helloworld_py helloworld
```
