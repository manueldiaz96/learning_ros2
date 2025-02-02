import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimpleSusbscriber(Node):
    def __init__(self):
        super().__init__(node_name="simple_subscriber")
        
        self.sub_ = self.create_subscription(String, "chatter", self.msgCallback, 10)
        
    def msgCallback(self, msg):
        self.get_logger().info("I heard {}".format(msg.data))
        

def main():
    rclpy.init()
    simple_subscriber = SimpleSusbscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()        


if __name__ == "__main__":
    main()