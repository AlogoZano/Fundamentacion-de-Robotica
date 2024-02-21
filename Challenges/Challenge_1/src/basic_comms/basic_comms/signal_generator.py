import rclpy
import math as ma
from rclpy.node import Node
from std_msgs.msg import Float32


class Signal_Generator(Node):

    def __init__(self):
        super().__init__('signal_generator_node')
        self.publisher_signal = self.create_publisher(Float32, 'signal', 10)
        self.publisher_time = self.create_publisher(Float32, 'time', 10)

        self.timer_period = 0.1
        self.time = 0

        self.msg_signal = Float32()
        self.msg_time = Float32()

        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Signal Generator node successfully initialized!')

    def timer_callback(self):
        self.time += self.timer_period

        self.msg_signal.data = ma.sin(self.time) 
        self.publisher_signal.publish(self.msg_signal)

        self.msg_time.data = self.time
        self.publisher_time.publish(self.msg_time)
        #self.get_logger().info('Tiempo: {} Senial: {}'.format(self.time, self.msg_signal.data))


def main(args = None):
    rclpy.init(args=args)
    m_s = Signal_Generator()
    rclpy.spin(m_s) #While(1)
    m_s.destroy_node() #Destruir nodo
    rclpy.shutdown()    

if __name__ == '__main__':
    main()
