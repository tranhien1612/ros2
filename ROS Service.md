# ROS2 service

## Introduction to service communication
Service communication is a communication model based on request and response. Between the two communicating parties, the client sends request data to the server, and the server responds to the client.

The client/server model is as follows:

![](http://www.yahboom.net/public/upload/upload-html/1713175152/image8.gif)

## Create a new function package

```
cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src
ros2 pkg create pkg_service --build-type ament_python --dependencies rclpy --node-name server_demo
```

## Server-side implementation

Next edit ```server_demo.py``` to implement server-side functions and add the following code:

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

Let’s first take a look at what the data of the AddTwoInts type looks like. You can use the following command to view it.

```
ros2 interface show example_interfaces/srv/AddTwoInts
```

## Client implementation
Create a new file ```client_demo.py``` in the same directory as ```server_demo.py```

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

## Edit configuration file

![](http://www.yahboom.net/public/upload/upload-html/1713175152/image-20231023185118116.png)

## Compile workspace

```
cd ~/yahboomcar_ros2_ws/yahboomcar_ws
colcon build --packages-select pkg_service
source install/setup.bash
```

## Run program

```
#Start server node
ros2 run pkg_service server_demo
#Start client node
ros2 run pkg_service client_demo
```



