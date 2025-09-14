import rclpy
from rclpy.node import Node

from std_msgs.msg import String
import time

acknowledged = False

class LAPTOP(Node):

    def __init__(self):
        super().__init__('laptop')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.subscription = self.create_subscription(
            String,
            'topic',
            self.mode_callback,
            10)

        self.mode = 'maze'

    def publish_mode(self):
        # while acknowledged == False:
            msg = String()
            #maybe like make sure both programs communicating then publish maze
            msg.data = self.mode
            self.publisher_.publish(msg)
            self.get_logger().info('Publishing: "%s"' % msg.data)

    def mode_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

        if msg.data.startswith('ACK '):
            acknowledged = True
        
        if msg.data == 'explore':
            self.mode = msg.data
            # self.publish_mode() might put in some acknowledging

        self.get_logger().info('Current mode %s' % self.mode)
    
    def do_smth(self):
        while True:
            counter = 0
            while self.mode == 'maze' and counter < 2:
                rclpy.spin_once(LAPTOP)
                print('Maze mode')
                time.sleep(3)
                counter += 1
            
            if self.mode == 'maze': #switch mode
                self.mode = 'line'
                print('Line Mode')
                self.publish_mode()
        
        




def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = LAPTOP()

    # rclpy.spin(minimal_publisher)
    minimal_publisher.do_smth()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
