import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time

class RPI(Node):

    def __init__(self):
        super().__init__('laptop')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.mode_callback,
            10)
        self.subscription  # prevent unused variable warning

    def publish_mode(self, mode):
        msg = String()
        #maybe like make sure both programs communicating then publish maze
        msg.data = mode
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def mode_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        if msg.data == 'line':
            self.mode = msg.data
            self.publish_mode('ACK ')

        self.get_logger().info('Current mode %s' % self.mode)
    
    def do_smth(self):
        # rclpy.spin_once(RPI)
        self.publish_mode('explore')
        self.publish_mode('explore')
        # while True:
        #     counter = 0
        #     while self.mode == 'line' and counter < 2:
        #         rclpy.spin_once(RPI)
        #         print('Line mode')
        #         time.sleep(3)
        #         counter += 1
            
        #     if self.mode == 'line': #switch mode
        #         self.mode = 'explore'
        #         print('Explore Mode')
        #         self.publish_mode()
        
        




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RPI()

    # rclpy.spin(minimal_publisher)
    minimal_publisher.do_smth()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
