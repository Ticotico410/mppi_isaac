#!/usr/bin/env python3
"""
Gazebo-only MPPI Simulator Interface
Works with ROS2 Gazebo via socket communication
"""

import torch
import zerorpc
import time
import socket
import threading
import json
import io
from dataclasses import dataclass
from abc import ABC, abstractmethod

def torch_to_bytes(t: torch.Tensor) -> bytes:
    buff = io.BytesIO()
    torch.save(t, buff)
    buff.seek(0)
    return buff.read()


def bytes_to_torch(b: bytes) -> torch.Tensor:
    buff = io.BytesIO(b)
    return torch.load(buff)


@dataclass
class GazeboSimStates:
    dof_state = None
        
class GazeboSimulator:
    """Gazebo simulator via socket communication"""

    def __init__(self):
        self.socket_available = False
        self.socket = None
        self.sim = GazeboSimStates()
        self.dt = 0.01
        self.cfg = None
        self.planner = None
        self.index_order = None
        
    def initialize(self, cfg):
        """Connect to Gazebo via ROS2 Simulator Bridge"""
        self.cfg = cfg
        
        # Connect to MPPI planner
        print("Connecting to MPPI planner...")
        self.planner = zerorpc.Client()
        self.planner.connect("tcp://127.0.0.1:4243")
        print("MPPI server found!")
        
        print("Using MPPI planner's existing Isaac Gym simulation for tensor generation")
        
        # Try to connect to ROS2 Simulator Bridge
        print("Connecting to ROS2 Simulator Bridge...")
        try:
            # Connect to ROS2 Simulator Bridge socket
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect(('localhost', 9090))  # ROS2 Simulator Bridge port
            
            # Initialize joint state storage
            self.sim.dof_state = torch.zeros(1, 12, device="cuda:0")
            # Start joint state listener thread
            self.joint_state_thread = threading.Thread(target=self._joint_state_listener)
            self.joint_state_thread.daemon = True
            self.joint_state_thread.start()
            
            self.socket_available = True
            print("Connected to ROS2 Simulator Bridge")
            
        except Exception as e:
            print(f"Failed to connect to ROS2 Simulator Bridge: {e}")
            print("Make sure ROS2 Simulator Bridge is running on port 9090")
            self.socket_available = False
            # Initialize joint state storage

    def _joint_state_listener(self):
        """Listen for joint state messages from socket"""
        buffer = ""
        time.sleep(1)
        
        while self.socket_available:
            try:
                data = self.socket.recv(1024)
                if not data:
                    break
                
                buffer += data.decode()
                
                # Process complete JSON messages
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    if line.strip():
                        try:
                            message = json.loads(line.strip())
                            
                            if message.get('type') == 'joint_states':
                                positions = message.get('data', {}).get('positions', [])
                                velocities = message.get('data', {}).get('velocities', [])
                                
                                # Map Gazebo order [2,3,1,4,5,6] to DOF order [1,2,3,4,5,6]
                                if len(positions) >= 6:
                                    self.sim.dof_state[0, 0] = positions[2]   # joint1
                                    self.sim.dof_state[0, 1] = velocities[2]  # joint1 vel
                                    self.sim.dof_state[0, 2] = positions[0]   # joint2
                                    self.sim.dof_state[0, 3] = velocities[0]   # joint2 vel
                                    self.sim.dof_state[0, 4] = positions[1]   # joint3
                                    self.sim.dof_state[0, 5] = velocities[1]   # joint3 vel
                                    self.sim.dof_state[0, 6] = positions[3]   # joint4
                                    self.sim.dof_state[0, 7] = velocities[3]   # joint4 vel
                                    self.sim.dof_state[0, 8] = positions[4]   # joint5
                                    self.sim.dof_state[0, 9] = velocities[4]   # joint5 vel
                                    self.sim.dof_state[0, 10] = positions[5]  # joint6
                                    self.sim.dof_state[0, 11] = velocities[5]  # joint6 vel
                        except json.JSONDecodeError:
                            continue
                        
            except Exception as e:
                print(f"Error in joint state listener: {e}")
                break
        
    def add_objects(self, objects):
        """Add objects to MPPI planner only"""
        if self.planner:
            self.planner.add_to_env(objects)

    def create_objects(self):
        """Create objects for the simulation"""
        # Object configuration
        obj_index = -1
        baseline = 2
        baseline2_pose = 'left'
        baseline1_pose = 1
        
        # Define goal pose
        if baseline == 2:
            if baseline2_pose == 'left':
                goal_pose = [0.7, 0.2, 0.5, 0, 0, 0.258819, 0.9659258]
            elif baseline2_pose == 'center':
                goal_pose = [0.65, 0, 0.5, 0, 0, 0, 1]
            else:  # right
                goal_pose = [0.7, -0.2, 0.5, 0, 0, -0.258819, 0.9659258]
        else:
            if baseline1_pose == 0:
                goal_pose = [0.5, 0.3, 0.5, 0.0, 0.0, 0.0, 1.0]
            else:
                goal_pose = [0.4, 0.3, 0.5, 0, 0, -0.7071068, 0.7071068]
        
        # Object parameters
        obj_set = [
            [0.100, 0.100, 0.05, 0.200, 0.250, 0.4, 0.],     # Baseline 1, pose 1
            [0.105, 0.063, 0.063, 0.20, 0.250, 0.42, 0.],     # Hageslag
            [0.116, 0.116, 0.06, 0.637, 0.016, 0.38, 0.],     # Baseline 2, A
            [0.168, 0.237, 0.05, 0.232, 0.615, 0.40, 0.],     # Baseline 2, B
            [0.198, 0.198, 0.06, 0.198, 0.565, 0.40, 0.],     # Baseline 2, C
            [0.166, 0.228, 0.08, 0.312, 0.587, 0.39, 0.],     # Baseline 2, D
            [0.153, 0.462, 0.05, 0.181, 0.506, 0.37, 0.],     # Baseline 2, E
            [0.162, 0.086, 0.068, 0.300, 0.25, 0.38, 0.]
        ]
        
        obj_ = obj_set[obj_index][:]
        table_dim = [1, 1, 0.108]
        table_pos = [0.5, 0., table_dim[-1]/2]
        
        # Create objects
        objects = [
            {
                "type": "box",
                "name": "obj_to_push",
                "size": [obj_[0], obj_[1], obj_[2]],
                "init_pos": [0.6, 0.2, table_dim[-1] + obj_[2] / 2],
                "mass": obj_[4],
                "fixed": False,
                "handle": None,
                "color": [0.2, 0.2, 0.8],
                "friction": obj_[3],
            },
            {
                "type": "box",
                "name": "table",
                "size": table_dim,
                "init_pos": table_pos,
                "fixed": True,
                "handle": None,
                "color": [255 / 255, 120 / 255, 57 / 255],
                "friction": obj_[3],
            },
            {
                "type": "box",
                "name": "goal",
                "size": [obj_[0], obj_[1], 0.005],
                "init_pos": [goal_pose[0], goal_pose[1], table_dim[-1]],
                "init_ori": [goal_pose[3], goal_pose[4], goal_pose[5], goal_pose[6]],
                "fixed": True,
                "color": [119 / 255, 221 / 255, 119 / 255],
                "handle": None,
                "collision": False,
            }
        ]
        
        return objects
    
    def set_action(self, action):
        """Send joint commands to Gazebo via ROS2 Simulator Bridge"""
        if not self.socket_available:
            return
            
        try:
            # Create action command message for ROS2 bridge
            joint_commands = action.cpu().numpy().tolist()
            message = {
                'type': 'action_command',
                'data': joint_commands
            }
            
            # Send message to ROS2 bridge
            self.socket.send((json.dumps(message) + '\n').encode())
            
        except Exception as e:
            print(f"Error sending joint commands to ROS2 bridge: {e}")
        
    def step(self):
        """Gazebo runs at its own pace"""
        if self.socket_available:
            time.sleep(self.dt)
    
    def sync_with_mppi(self):
        """Sync with MPPI planner using a specific instance for efficiency"""
        if self.planner:
            #TODO figure out how to rollout states into isaacgym
            self.planner.reset_rollout_only_robot(torch_to_bytes(self.sim.dof_state[0]))
            # print('state vector looks like this:', self.sim.dof_state[0])    
           
    def get_timestep(self):
        """Get Gazebo timestep"""
        return self.dt
        
    def cleanup(self):
        """Clean up"""
        if self.planner:
            self.planner.close()
        if self.socket_available:
            self.socket_available = False
            if self.socket:
                self.socket.close()


class SimulatorController:
    """Controller that orchestrates the simulation"""
    
    def __init__(self, simulator):
        self.simulator = simulator
        self.n_trials = 0
        
    def initialize(self, cfg):
        """Initialize the controller"""
        self.cfg = cfg
        self.simulator.initialize(cfg)

    def run_control_loop(self):
        """Run the main control loop"""
        print("Starting Gazebo control loop...")
        
        # Add objects to simulation
        objects = self.simulator.create_objects()  # Add your objects here
        self.simulator.add_objects(objects)  
        # Main control loop
        while self.n_trials < 1000:  # Run for 1000 steps
            t = time.time()
            
            # Sync simulator with MPPI planner
            self.simulator.sync_with_mppi()
            
            # Compute action from MPPI
            try:
                action = bytes_to_torch(self.simulator.planner.command())
            except Exception as e:
                print("Error: ", e)
                action = torch.zeros(6, device="cuda:0")
            print('action looks like this:', action)
            print('state looks like this:', self.simulator.sim.dof_state[0])
            
            # Apply action and step simulation
            self.simulator.set_action(action)
            self.simulator.step()
            
            # Update timing
            dt = self.simulator.get_timestep()
            print(f"Step {self.n_trials}: FPS: {1/(time.time() - t):.1f} RT-factor: {dt/(time.time() - t):.3f}")
            
            self.n_trials += 1
            
            # Check for timeout
            if self.n_trials > 1000:
                print("Timeout reached!")
                break
                
    def cleanup(self):
        """Clean up resources"""
        self.simulator.cleanup()


def main():
    """Main function to run Gazebo simulation"""
    # Create minimal config
    class MinimalConfig:
        def __init__(self):
            self.actors = ['xarm6_stick']
            self.initial_actor_positions = [[0.0, 0.0, 0.0]]
    
    cfg = MinimalConfig()
    
    # Create Gazebo simulator
    simulator = GazeboSimulator()
    
    # Create controller
    controller = SimulatorController(simulator)
    
    try:
        # Initialize and run
        controller.initialize(cfg)
        controller.run_control_loop()
    finally:
        controller.cleanup()


if __name__ == "__main__":
    main()
