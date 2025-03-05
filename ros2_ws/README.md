# ROS2

## Create workspace
```
mkdir ros2_ws
mkdir -p ros2_ws/src
```

## Build workspace
```
cd ros2_ws
colcon build
source /opt/ros/humble/setup.bash
```

## Clean workspace
```
sudo apt install python3-colcon-clean
colcon clean workspace
```

## ROS2 Node
### Creat package
```
cd src
# Create C++ function package
ros2 pkg create pkg_helloworld_cpp --build-type ament_cmake --dependencies rclcpp --node-name helloworld  
# Create Python feature package
ros2 pkg create pkg_helloworld_py --build-type ament_python --dependencies rclpy --node-name helloworld
```

### Build package
```
cd ros2_ws
colcon build --packages-select pkg_helloworld_py
```

### Edit ROS2 Node
Edit ```src/pkg_helloworld_py/helloworld.py```:

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
    rclpy.shutdown()  
```

### Run the node
```
source install/setup.bash
ros2 run pkg_helloworld_py helloworld
```

## ROS2 Topic

### Create package
```
cd ros2_ws/src
ros2 pkg create pkg_topic --build-type ament_python --dependencies rclpy --node-name publisher_demo
```

### Create Publisher Node
Edit ```publisher_demo.py```:
```
#Import rclpy library
import rclpy
from rclpy.node import Node
#Import String message
from std_msgs.msg import String 
#Create a Topic_Pub node subclass that inherits from the Node base class and pass in a parameter name
class Topic_Pub(Node):
    def __init__(self,name):
        super().__init__(name)
        #To create a publisher, use the create_publisher function. The parameters passed in are:
        #Topic data type, topic name, queue length to save messages
        self.pub = self.create_publisher(String,"/topic_demo",1) 
        #Create a timer and enter the interrupt processing function at intervals of 1s. The parameters passed in are:
        #Interval time between interrupt function execution, interrupt processing function
        self.timer = self.create_timer(1,self.pub_msg)
    #Define interrupt handler function
    def pub_msg(self):
        msg = String()  #Create a variable msg of type String
        msg.data = "Hi,I send a message." #Assign a value to the data in msg
        self.pub.publish(msg) #Publish topic data
        
#Main function
def main():
    rclpy.init() #initialization
    pub_demo = Topic_Pub("publisher_node") #Create a Topic_Pub class object, and the parameter passed in is the name of the node.
    rclpy.spin(pub_demo)     #Execute the rclpy.spin function and pass in a parameter. The parameter is the Topic_Pub class object just created.
    pub_demo.destroy_node()  #Destroy node object
    rclpy.shutdown()         #Turn off the ROS2 Python interface
```

### Create Subcriber Node
Create a new file ```subscriber_demo.py``` in the same directory as ```publisher_demo.py```:
```
#Import related libraries
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Topic_Sub(Node):
    def __init__(self,name):
        super().__init__(name)  
        #To create a subscriber, create_subscription is used. The parameters passed in are: topic data type, topic name, callback function name, and queue length.
        self.sub = self.create_subscription(String,"/topic_demo",self.sub_callback,1) 
    #Callback function execution program: print the received information
    def sub_callback(self,msg):
        print(msg.data)
def main():
    rclpy.init() #ROS2 Python interface initialization
    sub_demo = Topic_Sub("subscriber_node") # Create the object and initialize it
    rclpy.spin(sub_demo)
    sub_demo.destroy_node()  #Destroy node object
    rclpy.shutdown()         #Close the ROS2 Python interface
```

### Edit configuration file
Edit configuration file ```src/pkg_topic/setup.py ```:
```
'publisher_demo = pkg_topic.publisher_demo:main'
'subscriber_demo = pkg_topic.subscriber_demo:main'
```

### Build package
```
cd ros2_ws
colcon build --packages-select pkg_topic
```

### Run program
```
#Start publisher node
ros2 run pkg_topic publisher_demo
#Start Subscriber Node
ros2 run pkg_topic subscriber_demo
```

### Check topic published, and open another terminal to input
```
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic echo /topic_demo
```

## ROS2 Service
### Create package
```
cd ros2_ws/src
ros2 pkg create pkg_service --build-type ament_python --dependencies rclpy --node-name server_demo
```

### Edit Server Node
```
#Import related library files
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service_Server(Node):
    def __init__(self,name):
        super().__init__(name)
        #To create a server, use the create_service function, and the parameters passed in are:
         #The data type of service data, the name of the service, and the service callback function (that is, the content of the service)
        self.srv = self.create_service(AddTwoInts, '/add_two_ints', self.Add2Ints_callback)
    #The content of the service callback function here is to add two integers and then return the added result. 
    def Add2Ints_callback(self,request,response):
        response.sum = request.a + request.b
        print("response.sum = ",response.sum)
        return response
def main():
    rclpy.init()
    server_demo = Service_Server("publisher_node")
    rclpy.spin(server_demo)
    server_demo.destroy_node()                     # Destroy node object
    rclpy.shutdown()                               # Close the ROS2 Python interface
```

### Create Client Service
Create a new file ```client_demo.py``` in the same directory as ```server_demo.py```:
```
#Import related libraries
import rclpy
from rclpy.node import Node
from example_interfaces.srv import AddTwoInts

class Service_Client(Node):
    def __init__(self,name):
        super().__init__(name)
        #To create a client, use the create_client function. The parameters passed in are the data type of the service data and the topic name of the service.
        self.client = self.create_client(AddTwoInts,'/add_two_ints')
        # Loop waiting for the server to start successfully
        while not self.client.wait_for_service(timeout_sec=1.0):
            print("service not available, waiting again...")
        # Create data objects for service requests
        self.request = AddTwoInts.Request()
        
    def send_request(self): 
        self.request.a = 10
        self.request.b = 90
        #send service request
        self.future = self.client.call_async(self.request)
        
def main():
    rclpy.init() #Node initialization
    service_client = Service_Client("client_node") #Create object
    service_client.send_request() #send service request
    while rclpy.ok():
        rclpy.spin_once(service_client)
        #Determine whether data processing is completed
        if service_client.future.done():
            try:
                #Get service feedback information and print it
                response = service_client.future.result()
                print("service_client.request.a = ",service_client.request.a)
                print("service_client.request.b = ",service_client.request.b)
                print("Result = ",response.sum)
            except Exception as e:
                service_client.get_logger().info('Service call failed %r' % (e,))
        break
    service_client.destroy_node()                    
    rclpy.shutdown()      
```

### Show AddTwoInts type
```
ros2 interface show example_interfaces/srv/AddTwoInts
```

### Edit configuration file
Edit configuration file ```src/pkg_service/setup.py ```:
```
'server_demo = pkg_service.server_demo:main',
'client_demo = pkg_service.client_demo:main'
```

### Build package
```
cd ros2_ws
colcon build --packages-select pkg_service
source install/setup.bash
```

### Run program
```
#Start server node
ros2 run pkg_service server_demo
#Start client node
ros2 run pkg_service client_demo
```

### Show Service List
```
ros2 service list
```

## ROS2 Action
### Create action communication interface function package
#### Create package
1. Create package
```
cd ros2_ws/src
ros2 pkg create --build-type ament_cmake pkg_interfaces
```

2. Create ```action``` folder and ```Fibonacci.action``` file in the action folder. The file content is as follows:
```
int32 order
---
int32[] sequence
---
int32[] partial_sequence
```

3. Some dependency packages need to be added to ```src/pkg_interfaces/package.xml```. The specific contents are as follows:
```
<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>
```

4. Add the following configuration to ```src/pkg_interfaces/CMakeLists.txt```:
```
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)
```

#### Build package
```
cd ros2_ws
colcon build --packages-select pkg_interfaces
```

#### Check the file definition
```
source install/setup.bash
ros2 interface show pkg_interfaces/action/Fibonacci
```

### Create action communication function package
#### Create package
```
cd ros2_ws/src
ros2 pkg create pkg_action --build-type ament_python --dependencies rclpy pkg_interfaces --node-name action_server_demo
```

#### Edit Action Server File
Next edit ```action_server_demo.py```:
```
import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from pkg_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self):
        super().__init__('fibonacci_action_server')
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            self.execute_callback)

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)

    fibonacci_action_server = FibonacciActionServer()

    rclpy.spin(fibonacci_action_server)


if __name__ == '__main__':
    main()
``` 

#### Create Action Client File
Create ```action_client_demo.py``` in the same directory as ```action_server_demo.py```:
```
from pkg_interfaces.action import Fibonacci

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class FibonacciActionClient(Node):

    def __init__(self):
        super().__init__('fibonacci_action_client')
        self._action_client = ActionClient(self, Fibonacci, 'fibonacci')

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()

        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))


def main(args=None):
    rclpy.init(args=args)

    action_client = FibonacciActionClient()

    action_client.send_goal(10)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()
```

#### Edit configuration file
Edit ```src/pkg_action/setup.py```file:
```
'action_server_demo = pkg_action.action_server_demo:main',
'action_client_demo = pkg_action.action_client_demo:main'
```

#### Build package
```
cd ros2_ws
colcon build --packages-select pkg_action
source install/setup.bash
```

#### Run program
```
ros2 run pkg_action action_server_demo
ros2 run pkg_action action_client_demo
```

#### Show action list
```
ros2 action list
```