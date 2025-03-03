# ROS2 topic

## Create a new function package

```
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src
ros2 pkg create pkg_topic --build-type ament_python --dependencies rclpy --node-name publisher_demo
```

## Publisher implementation

Next edit ```publisher_demo.py``` to implement the publisher’s functions and add the following code:

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
Create a new file ```subscriber_demo.py``` in the same directory as ```publisher_demo.py```.

Next edit ```subscriber_demo.py``` to implement the subscriber function and add the following code:

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
## Edit configuration file

![](http://www.yahboom.net/public/upload/upload-html/1713175136/image-20231023175338754.png)

## Compile workspace
```
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select pkg_topic
source install/setup.bash
```

## Run program

```
#Start publisher node
ros2 run pkg_topic publisher_demo
#Start Subscriber Node
ros2 run pkg_topic subscriber_demo
```

