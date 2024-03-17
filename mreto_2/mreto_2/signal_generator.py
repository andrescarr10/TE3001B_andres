import rclpy
from rclpy.node import Node
import numpy as np
from scipy import signal
from std_msgs.msg import String, Int32, Float32
from srv_int.msg import SignalData

class Signal_Generator(Node):
    def __init__(self):
        super().__init__('signal_generator_node')
        self.signal_publisher_ = self.create_publisher(Float32, 'signal', 10)
        self.signal_parameters_publisher_ = self.create_publisher(SignalData, 'signal_params', 10)
        self.declare_parameters(
                namespace='',
                parameters=[('type_of_signal', rclpy.Parameter.Type.INTEGER),('amplitude_param', rclpy.Parameter.Type.DOUBLE),('frequency_param', rclpy.Parameter.Type.DOUBLE),('offset_param', rclpy.Parameter.Type.DOUBLE),
                ]
        )
        self.signal_handler_period = 0.01
        self.signal_handler = self.create_timer(self.signal_handler_period, self.signal_handler_callback)
        self.signal_variable = Float32() 
        self.signal_params_variable = SignalData()
        self.time_variable = 0
        self.get_logger().info('signal_generator_node Initialized!!!')

    def signal_handler_callback(self):
        self.time_variable += self.signal_handler_period
        type_of_signal_variable = self.get_parameter('type_of_signal').get_parameter_value().integer_value
        amplitude_variable = self.get_parameter('amplitude_param').get_parameter_value().double_value
        frequency_var = self.get_parameter('frequency_param').get_parameter_value().double_value
        offset_var = self.get_parameter('offset_param').get_parameter_value().double_value
        self.signal_params_variable.type = type_of_signal_variable
        self.signal_params_variable.amplitude = amplitude_variable
        self.signal_params_variable.frequency = frequency_var
        self.signal_params_variable.offset = offset_var

        freq_to_rad_Persecond = (self.time_variable * frequency_var * np.pi * 2)
        if(type_of_signal_variable == 1):
            self.signal_variable.data = np.sin(freq_to_rad_Persecond + offset_var) * amplitude_variable        
        elif(type_of_signal_variable == 2):
            t = [freq_to_rad_Persecond + offset_var]
            self.signal_variable.data = signal.square(t, duty=0.5)[0] * amplitude_variable
        elif(type_of_signal_variable == 3):
            t = [freq_to_rad_Persecond + offset_var]
            self.signal_variable.data = signal.sawtooth(t)[0] * amplitude_variable
        elif(type_of_signal_variable == 4):
            t = [freq_to_rad_Persecond + offset_var]
            self.signal_variable.data = (signal.square(t, duty=0.5)[0] * amplitude_variable) + (signal.sawtooth(t)[0] * amplitude_variable)
        elif(type_of_signal_variable == 5):
            t = [freq_to_rad_Persecond + offset_var]
            self.signal_variable.data = (signal.square(t, duty=0.5)[0] * amplitude_variable) - (signal.sawtooth(t)[0] * amplitude_variable)

        
        self.signal_publisher_.publish(self.signal_variable)
        self.signal_parameters_publisher_.publish(self.signal_params_variable)

def main(args=None):
    rclpy.init(args=args)
    node = Signal_Generator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

