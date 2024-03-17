import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class Process(Node):
    processed_signal_msg = Float32()
    def __init__(self):
        super().__init__('process')
        self.processed_signal_publisher_ = self.create_publisher(Float32, 'proc_signal', 10)
        timer_period = 0.1 
        self.time_subscriber_ = self.create_subscription(Float32, 'time', self.time_callback, 10)
        self.signal_subscriber_ = self.create_subscription(Float32, 'signal', self.signal_callback, 10)
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0

    def timer_callback(self):
        self.processed_signal_publisher_.publish(self.processed_signal_msg)

    def signal_callback(self, msg):
        self.get_logger().info('Original Signal= "%f", Processed Signal= "%f"' % (msg.data, self.processed_signal_msg.data))

    def time_callback(self, msg):
        self.processed_signal_msg.data = (math.sin(msg.data * 10) + 1) * 0.5 

def main(args=None):
    rclpy.init(args=args)
    pro = Process()
    rclpy.spin(pro)
    pro.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()