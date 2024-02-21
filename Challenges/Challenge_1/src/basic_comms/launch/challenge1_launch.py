import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    signal_node  = Node(package = 'basic_comms',  
                        executable='signal_generator',
                        output = 'screen')
    processed_signal_node  = Node(package = 'basic_comms',    
                                  executable='process',   
                                  output = 'screen')
    rqt_plot_node  = Node(package = 'rqt_plot',    
                          executable='rqt_plot',    
                          output = 'screen')
    rqt_graph_node  = Node(package = 'rqt_graph',    
                          executable='rqt_graph',    
                          output = 'screen')
    ld = LaunchDescription ([signal_node,processed_signal_node,rqt_plot_node,rqt_graph_node])
    return ld
    