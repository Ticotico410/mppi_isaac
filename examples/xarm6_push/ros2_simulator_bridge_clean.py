#!/usr/bin/env python3
"""
ROS2 Simulator Bridge - Clean minimal version
Connects ROS2 Gazebo to MPPI Simulator Interface
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from gazebo_msgs.msg import ModelStates
import socket
import json
import threading
import time

class ROS2SimulatorBridge(Node):
    def __init__(self):
        super().__init__('ros2_simulator_bridge')
        
        # Publishers and subscribers
        self.velocity_pub = self.create_publisher(Float64MultiArray, '/xarm6_velocity_controller/commands', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Subscribe to Gazebo model states
        self.model_states_sub = self.create_subscription(ModelStates, '/model_states', self.model_states_callback, 10)
        
        
        # Socket server
        self.simulator_client = None
        self.simulator_connected = False
        self.latest_joint_states = None
        self.latest_block_pose = None
        
        # Start socket server
        self.start_socket_server()
        
        # Continuous joint state broadcasting at 100Hz
        self.timer = self.create_timer(0.01, self.send_joint_states)
        
        # Block pose broadcasting at 10Hz
        self.block_pose_timer = self.create_timer(0.1, self.send_block_pose)
        
        # Initialize with default block pose
        self.latest_block_pose = None
        

    def start_socket_server(self):
        """Start socket server for simulator interface"""
        self.socket_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.socket_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.socket_server.bind(('localhost', 9090))
        self.socket_server.listen(1)
        
        self.server_thread = threading.Thread(target=self._handle_connections)
        self.server_thread.daemon = True
        self.server_thread.start()

    def _handle_connections(self):
        """Handle connections from simulator interface"""
        while True:
            self.simulator_client, addr = self.socket_server.accept()
            self.simulator_connected = True
            self._handle_messages()

    def _handle_messages(self):
        """Handle messages from simulator interface"""
        while self.simulator_connected:
            data = self.simulator_client.recv(1024)
            if not data:
                break
                
            message = json.loads(data.decode())
            
            if message.get('type') == 'action_command':
                self._apply_action(message.get('data', []))

    def joint_state_callback(self, msg):
        """Handle joint states from ROS2"""
        self.latest_joint_states = {
            'positions': list(msg.position),
            'velocities': list(msg.velocity),
            'joint_names': list(msg.name)
        }

    def model_states_callback(self, msg):
        """Extract block pose from model states"""
        for i, name in enumerate(msg.name):
            if name == 'block':
                block_pose = msg.pose[i]
                self.latest_block_pose = {
                    'position': [block_pose.position.x, block_pose.position.y, block_pose.position.z],
                    'orientation': [block_pose.orientation.x, block_pose.orientation.y, 
                                  block_pose.orientation.z, block_pose.orientation.w]
                }
                break


    def send_joint_states(self):
        """Send joint states to simulator interface"""
        if not self.latest_joint_states or not self.simulator_connected:
            return
            
        message = {
            'type': 'joint_states',
            'data': {
                'positions': self.latest_joint_states['positions'],
                'velocities': self.latest_joint_states['velocities'],
                'joint_names': self.latest_joint_states['joint_names']
            }
        }
        
        self.simulator_client.send((json.dumps(message) + '\n').encode())

    def _apply_action(self, action_data):
        """Apply action to Gazebo via velocity commands"""
        if len(action_data) != 6:
            return
            
        velocity_msg = Float64MultiArray()
        velocity_msg.data = list(action_data)
        self.velocity_pub.publish(velocity_msg)


    def send_block_pose(self):
        """Send block pose to simulator interface via socket"""
        if not self.latest_block_pose or not self.simulator_connected:
            return
            
        message = {
            'type': 'block_pose',
            'data': {
                'position': self.latest_block_pose['position'],
                'orientation': self.latest_block_pose['orientation']
            }
        }
        
        self.simulator_client.send((json.dumps(message) + '\n').encode())

    def destroy_node(self):
        """Clean up when node is destroyed"""
        if hasattr(self, 'socket_server'):
            self.socket_server.close()
        if self.simulator_client:
            self.simulator_client.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    try:
        bridge = ROS2SimulatorBridge()
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
