import rclpy
from rclpy.node import Node #Este Node sirve para crear un nodo
from std_msgs.msg import Float32
import math as ma

class Process(Node):
    def __init__(self):
        super().__init__('process')

        self.proc_signal_publisher = self.create_publisher(Float32, 'proc_signal', 10)
        self.msg_proc = Float32()

        self.amplitud = 0.0
        self.media_amplitud = 0.0
        self.phaseshift = ma.pi
        self.offset = 1.5
        self.paso_offset = 0.1

        

        self.sub_signal = self.create_subscription(Float32, 
                                            'signal', 
                                            self.listener_signal_callback,
                                            10)
        
        self.sub_time = self.create_subscription(Float32, 
                                            'time', 
                                            self.listener_time_callback,
                                            10)
        
        self.get_logger().info('Process node successfully initialized!')
        
    def listener_signal_callback(self, msg):
        self.dato_act = msg.data
        self.amplitud = max(self.amplitud, self.dato_act)
        #self.offset += self.paso_offset
        #if(self.offset == self.amplitud):
            #self.paso_offset = -0.1
        #elif(self.offset == self.media_amplitud):
            #self.paso_offset = 0.1


    def listener_time_callback(self, msg):
        self.time = msg.data
        self.media_amplitud = 0.5 * self.amplitud
        self.processed_signal = self.media_amplitud*ma.sin(self.time + self.phaseshift) + self.offset

        self.msg_proc.data = self.processed_signal
        self.proc_signal_publisher.publish(self.msg_proc)



def main(args = None):
    rclpy.init(args=args)
    m_p = Process()
    rclpy.spin(m_p) #While(1)
    m_p.destroy_node() #Destruir nodo
    rclpy.shutdown()    

if __name__ == '__main__':
    main()