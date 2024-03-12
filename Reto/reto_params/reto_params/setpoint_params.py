import rclpy
import math as ma
from rclpy.node import Node
from std_msgs.msg import Float32
from scipy import signal
import numpy as np
#from setpoint_msg.msg import Setpoint


class Signal_Generator_node(Node):

    def __init__(self):
        super().__init__('setpoint_params_node')
        self.publisher_signal = self.create_publisher(Float32, 'setpoint', 10) #A cambiar por signal
        self.publisher_angular_speed = self.create_publisher(Float32, 'reference', 10)

        self.timer_period = 0.1
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        params = [
            ('signal_type', 1),
            ('amplitude', 1.0),
            ('offset', 0.0),
            ('frequency', 0.2)
            
        ]


        self.declare_parameters(
            namespace='', parameters=params)
        
        self.time = 0.0
        self.amp = 0.0
        self.msg_signal = Float32()
        self.msg_reference = Float32()

        

        self.get_logger().info('Signal Generator node successfully initialized!')

    def timer_callback(self):

        self.time += self.timer_period

        freq = self.get_parameter('frequency').get_parameter_value().double_value
        offset = self.get_parameter('offset').get_parameter_value().double_value
        omega = 2.0 * np.pi * freq
        
        self.amp = self.get_parameter('amplitude').get_parameter_value().double_value

        if(self.get_parameter('signal_type').get_parameter_value().integer_value == 1):
            self.msg_signal.data = (self.amp)*ma.sin(omega*self.time) + offset

        elif(self.get_parameter('signal_type').get_parameter_value().integer_value == 2):
            self.msg_signal.data = (self.amp)*signal.square(self.time * omega) + offset

        elif(self.get_parameter('signal_type').get_parameter_value().integer_value == 3):
            self.msg_signal.data = (self.amp)*signal.sawtooth(self.time * omega) + offset    
        

        self.publisher_signal.publish(self.msg_signal)
        self.msg_reference.data = self.msg_signal.data*6.8
        self.publisher_angular_speed.publish(self.msg_reference)

        #self.get_logger().info('Tiempo: {} Senial: {}'.format(self.time, self.msg_signal.data))

def main(args = None):
    rclpy.init(args=args)
    setpoint = Signal_Generator_node()
    rclpy.spin(setpoint) #While(1)
    setpoint.destroy_node() #Destruir nodo
    rclpy.shutdown()    

if __name__ == '__main__':
    main()