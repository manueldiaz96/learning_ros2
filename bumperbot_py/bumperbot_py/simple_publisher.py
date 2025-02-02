import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        
        self.pub_ = self.create_publisher(msg_type=String, topic='chatter', qos_profile=10)
        
        self.counter_ = 0
        self.frequency_ = 1.0
        
        self.get_logger().info("Publishing at {} Hz".format(self.frequency_))
        
        self.timer = self.create_timer(self.frequency_, self.timerCallback)
        
    def timerCallback(self):
        msg = String()
        msg.data = "Hello all - Counter: {}".format(self.counter_)
        
        self.pub_.publish(msg)
        
        self.counter_ += 1
        
def main():
    rclpy.init()
    my_publisher = SimplePublisher()
    rclpy.spin(my_publisher)
    my_publisher.destroy_node()
    rclpy.shutdown()
        
if __name__  == '__main__':
    main()