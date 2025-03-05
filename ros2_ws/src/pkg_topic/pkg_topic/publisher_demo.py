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