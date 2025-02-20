import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class MinimalPublisher(Node):
    def __init__(self):
        super().__init__('minimal_publisher')
        
        # this is the publisher object that we are going to be sending messages from
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        timer_period = 0.5  # seconds

        # create a timer that will call the timer_callback function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        
        # initialise the counter
        self.i = 0

    def timer_callback(self):
        # create a new message and set the contents
        msg = String()
        msg.data = f'Hello World: {self.i}'

        # publish the message to the topic we specified earlier
        self.publisher_.publish(msg)

        # logging and incrementing the counter
        self.get_logger().info(f'Publishing: "{msg.data}"')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    # the node will keep going, publishing messages whenever 
    #   the timer is triggered until the node is stopped
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()