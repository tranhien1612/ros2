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