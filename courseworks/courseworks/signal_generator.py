import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import math

class Signal_generator(Node):
    def __init__(self):
        super().__init__('signal_generator')
        self.signal_publisher_ = self.create_publisher(Float32, 'signal', 10)
        self.time_publisher_ = self.create_publisher(Float32, 'time', 10)
        timer_period = 0.1
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.x = 0.0

    def timer_callback(self):
        self.x += 0.01
        time_msg = Float32()
        time_msg.data = self.x
        self.time_publisher_.publish(time_msg)

        sine_msg = Float32()
        sine_msg.data = math.sin(math.radians(self.x * 800))
        self.signal_publisher_.publish(sine_msg)

        self.get_logger().info('Time: "%f", sin(t)= "%f"'%(time_msg.data, sine_msg.data))

def main(args=None):
    rclpy.init(args=args)
    s_g = Signal_generator()
    rclpy.spin(s_g)
    s_g.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
