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