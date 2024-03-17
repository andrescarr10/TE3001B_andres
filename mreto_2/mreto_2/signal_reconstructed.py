import rclpy
from rclpy.node import Node
import numpy as np
from scipy import signal
from std_msgs.msg import String, Int32, Float32
from srv_int.msg import SignalData

class Signal_Reconstructed(Node):
    def __init__(self):
        super().__init__('signal_reconstructed_node')
        self.signalGenerator_subscriber_ = self.create_subscription(SignalData, 'signal_params', self.signalgen_subscriber_callback, 10)
        self.signalReconstructed_publisher_ = self.create_publisher(Float32, 'signal_reconstructed', 10)
        self.signalrecon_handler_period = 0.01
        self.signalrecon_handler = self.create_timer(self.signalrecon_handler_period, self.signalrecon_handler_callback)
        self.signal_variable = Float32() 
        self.signal_params_variable = SignalData()
        self.signal_params_variable.type = 1
        self.signal_params_variable.amplitude = 0.0
        self.signal_params_variable.frequency = 0.0
        self.signal_params_variable.offset = 0.0
        self.time_variable = 0
        self.get_logger().info('signal_reconstructed_node Initialized!!!')

    def signalgen_subscriber_callback(self, msg):
         self.signal_params_variable = msg
        
    def signalrecon_handler_callback(self):
        self.time_variable += self.signalrecon_handler_period

        frequency2radiansPersecond = (self.time_variable * self.signal_params_variable.frequency * np.pi * 2)
        if(self.signal_params_variable.type == 1):
            self.signal_variable.data = np.sin(frequency2radiansPersecond + self.signal_params_variable.offset) * self.signal_params_variable.amplitude        
        elif(self.signal_params_variable.type == 2):
            t = [frequency2radiansPersecond + self.signal_params_variable.offset]
            self.signal_variable.data = signal.square(t, duty=0.5)[0] * self.signal_params_variable.amplitude
        elif(self.signal_params_variable.type == 3):
            t = [frequency2radiansPersecond + self.signal_params_variable.offset]
            self.signal_variable.data = signal.sawtooth(t)[0] * self.signal_params_variable.amplitude
        elif(self.signal_params_variable.type == 4):
            t = [frequency2radiansPersecond + self.signal_params_variable.offset]
            self.signal_variable.data = (signal.square(t, duty=0.5)[0] * self.signal_params_variable.amplitude) + (signal.sawtooth(t)[0] * self.signal_params_variable.amplitude)
        elif(self.signal_params_variable.type == 5):
            t = [frequency2radiansPersecond + self.signal_params_variable.offset]
            self.signal_variable.data = (signal.square(t, duty=0.5)[0] * self.signal_params_variable.amplitude) - (signal.sawtooth(t)[0] * self.signal_params_variable.amplitude)

        self.signalReconstructed_publisher_.publish(self.signal_variable)

def main(args=None):
    rclpy.init(args=args)
    node = Signal_Reconstructed()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

