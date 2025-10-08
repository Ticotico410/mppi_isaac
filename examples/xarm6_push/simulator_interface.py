import gym
from mppiisaac.planner.isaacgym_wrapper import IsaacGymWrapper, ActorWrapper
import numpy as np
import yaml
import mppiisaac
from yaml import SafeLoader
from mppiisaac.planner.mppi_isaac import MPPIisaacPlanner
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
import hydra
from omegaconf import OmegaConf
import os
import torch
import zerorpc
from mppiisaac.utils.config_store import ExampleConfig
from isaacgym import gymapi
import time
from client import Objective
import sys
import io
from abc import ABC, abstractmethod


def torch_to_bytes(t: torch.Tensor) -> bytes:
    buff = io.BytesIO()
    torch.save(t, buff)
    buff.seek(0)
    return buff.read()


def bytes_to_torch(b: bytes) -> torch.Tensor:
    buff = io.BytesIO(b)
    return torch.load(buff)


class SimulatorInterface(ABC):
    """Abstract base class for simulator interfaces"""
    
    @abstractmethod
    def initialize(self, cfg):
        """Initialize the simulator"""
        pass
    
    @abstractmethod
    def add_objects(self, objects):
        """Add objects to the simulation"""
        pass
    
    @abstractmethod
    def get_state(self):
        """Get current state of the simulation"""
        pass
    
    @abstractmethod
    def set_action(self, action):
        """Apply action to the simulation"""
        pass
    
    @abstractmethod
    def step(self):
        """Step the simulation forward"""
        pass
    
    @abstractmethod
    def reset(self, init_pos, init_vel):
        """Reset the simulation"""
        pass
    
    @abstractmethod
    def cleanup(self):
        """Clean up resources"""
        pass


class IsaacGymSimulator(SimulatorInterface):
    """Isaac Gym simulator implementation"""
    
    def __init__(self):
        self.sim = None
        self.cfg = None
        self.planner = None
        
    def initialize(self, cfg):
        """Initialize Isaac Gym simulation"""
        self.cfg = cfg
        
        # Connect to MPPI planner first
        self.planner = zerorpc.Client()
        self.planner.connect("tcp://127.0.0.1:4243")
        print("MPPI server found!")
        
        # Create a separate Isaac Gym simulation for visualization only
        # Load actors
        actors = []
        for actor_name in cfg.actors:
            with open(f'{os.path.dirname(mppiisaac.__file__)}/../conf/actors/{actor_name}.yaml') as f:
                actors.append(ActorWrapper(**yaml.load(f, Loader=SafeLoader)))
        
        # Create Isaac Gym simulation for visualization
        self.sim = IsaacGymWrapper(
            cfg.isaacgym,
            init_positions=cfg.initial_actor_positions,
            actors=actors,
            num_envs=1,
            viewer=True,  # Enable viewer for visualization
        )
        
    def add_objects(self, objects):
        """Add objects to Isaac Gym simulation"""
        # Add objects to both simulations
        self.sim.add_to_envs(objects)  # For visualization
        self.planner.add_to_env(objects)  # For MPPI computation
        
    def get_state(self):
        """Get current state - not needed since MPPI handles simulation internally"""
        # MPPI planner manages its own simulation state
        # We don't need to track state here
        return {}
    
    def set_action(self, action):
        """Apply action to simulation - MPPI handles this internally"""
        # MPPI planner handles action application internally
        pass
        
    def step(self):
        """Step simulation forward - MPPI handles this internally"""
        # MPPI planner handles stepping internally
        pass
        
    def reset(self, init_pos, init_vel):
        """Reset simulation"""
        if hasattr(self, 'sim'):
            self.sim.stop_sim()
            self.sim.start_sim()
            self.sim.gym.viewer_camera_look_at(
                self.sim.viewer, None, gymapi.Vec3(1.5, 2, 3), gymapi.Vec3(1.5, 0, 0)
            )
            self.sim.set_dof_state_tensor(torch.tensor([
                init_pos[0], init_vel[0], init_pos[1], init_vel[1], 
                init_pos[2], init_vel[2], init_pos[3], init_vel[3], 
                init_pos[4], init_vel[4], init_pos[5], init_vel[5]
            ], device="cuda:0"))
        
    def cleanup(self):
        """Clean up resources"""
        if self.sim:
            # IsaacGymWrapper doesn't have cleanup method
            # Just set to None for garbage collection
            self.sim = None
        if self.planner:
            self.planner.close()


class SimulatorController:
    """Main controller that uses any simulator interface"""
    
    def __init__(self, simulator: SimulatorInterface):
        self.simulator = simulator
        self.cfg = None
        self.client_helper = None
        self.init_time = None
        self.count = 0
        self.n_trials = 0
        self.timeout = 20
        self.data_time = []
        self.data_err = []
        self.data_rt = []
        self.rt_factor_seq = []
        
    def initialize(self, cfg):
        """Initialize the controller"""
        self.cfg = cfg
        self.simulator.initialize(cfg)
        self.client_helper = Objective(cfg, cfg.mppi.device)
        self.init_time = time.time()
        
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
                goal_pose = [0.3, 0.2, 0.5, 0, 0, 0.258819, 0.9659258]
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
            [0.162, 0.086, 0.068, 0.300, 0.25, 0.4, 0.3]
        ]
        
        obj_ = obj_set[obj_index][:]
        table_dim = [1, 1.0, 0.108]
        table_pos = [0.5, 0., table_dim[-1]/2]
        
        # Create objects
        objects = [
            {
                "type": "box",
                "name": "obj_to_push",
                "size": [obj_[0], obj_[1], obj_[2]],
                "init_pos": [obj_[5], obj_[6], table_dim[-1] + obj_[2] / 2],
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
    
    def run_control_loop(self):
        """Main control loop"""
        # Create and add objects
        objects = self.create_objects()
        self.simulator.add_objects(objects)
        
        # Set initial position #TODO this should be a part of simulation like sim.init_pos
        init_pos = [0.0, -0.785, -0.785, 0.0, 1.5708, 0.0]
        init_vel = [0., 0., 0., 0., 0., 0.]
        
        # Set initial state for visualization
        if hasattr(self.simulator, 'sim'):
            self.simulator.sim.set_dof_state_tensor(torch.tensor([
                init_pos[0], init_vel[0], init_pos[1], init_vel[1],
                init_pos[2], init_vel[2], init_pos[3], init_vel[3],
                init_pos[4], init_vel[4], init_pos[5], init_vel[5]
            ], device="cuda:0"))
        
        # Main control loop
        while self.n_trials < self.cfg.n_steps:
            t = time.time()
            
            # Reset MPPI planner with current state (like original server.py)
            if hasattr(self.simulator, 'planner') and hasattr(self.simulator, 'sim'):
                print('rollout shapes look like this:', self.simulator.sim.dof_state[0].shape, self.simulator.sim.root_state[0].shape, self.simulator.sim.rigid_body_state[0].shape)
                cube_obstacle_tensor = self.simulator.sim.root_state[0, 1, :]
                print('sim environment root tensor shape is', self.simulator.sim.root_state.shape)
                self.simulator.planner.update_root_state_tensor_by_obstacles_tensor(torch_to_bytes(cube_obstacle_tensor))
                self.simulator.planner.reset_rollout_only_robot(
                    torch_to_bytes(self.simulator.sim.dof_state[0])
                )
                self.simulator.sim.gym.clear_lines(self.simulator.sim.viewer)
            
            # Compute action from MPPI
            if hasattr(self.simulator, 'planner'):
                action = bytes_to_torch(self.simulator.planner.command())
                if torch.any(torch.isnan(action)):
                    print("NaN action detected")
                    action = torch.zeros_like(action)
            else:
                # Fallback: zero action
                action = torch.zeros(6, device="cuda:0")
            
            # Apply action to visualization simulation
            self.simulator.sim.set_dof_velocity_target_tensor(action)
            print('final angles are', self.simulator.sim.dof_state[0])
            # Step visualization simulation
            self.simulator.sim.step()
            
            # Check for success
            if self.count > 10:
                self._check_success(init_pos, init_vel)
            
            # Update timing
            self.rt_factor_seq.append(self.cfg.isaacgym.dt / (time.time() - t))
            print(f"FPS: {1/(time.time() - t)} RT-factor: {self.cfg.isaacgym.dt/(time.time() - t)}")
            
            # Check timeout
            if time.time() - self.init_time >= self.timeout:
                self.simulator.reset(init_pos, init_vel)
                self.init_time = time.time()
                self.count = 0
                self.n_trials += 1
            else:
                self.count += 1
    
    def _check_success(self, init_pos, init_vel):
        """Check if task is completed successfully"""
        if not hasattr(self.simulator, 'sim'):
            return
            
        # Get state from visualization simulation
        block_index = 1
        block_pos = self.simulator.sim.root_state[0, block_index, :3].unsqueeze(0)
        block_ort = self.simulator.sim.root_state[0, block_index, 3:7].unsqueeze(0)
        
        Ex, Ey, Etheta = self.client_helper.compute_metrics(block_pos, block_ort)
        metric_1 = 1.5 * (Ex + Ey) + 0.01 * Etheta
        
        print("Ex", Ex)
        print("Ey", Ey)
        print("Angle", Etheta)
        
        if Ex < 0.02 and Ey < 0.02 and Etheta < 0.1:
            print("Success!")
            final_time = time.time()
            time_taken = final_time - self.init_time
            print("Time to completion", time_taken)
            
            self.simulator.reset(init_pos, init_vel)
            self.init_time = time.time()
            self.count = 0
            self.data_rt.append(np.sum(self.rt_factor_seq) / len(self.rt_factor_seq))
            self.data_time.append(time_taken)
            self.data_err.append(np.float64(metric_1))
            self.n_trials += 1
            self.rt_factor_seq = []
    
    def print_results(self):
        """Print final results"""
        data_time = np.array(self.data_time)
        data_rt = np.array(self.data_rt)
        actual_time = data_time * data_rt
        data_err = np.array(self.data_err)
        
        if len(data_time) > 0:
            print("Num. trials", self.n_trials)
            print("Success rate", len(data_time) / self.n_trials * 100)
            print("Avg. Time", np.mean(actual_time))
            print("Std. Time", np.std(actual_time))
            print("Avg. error", np.mean(data_err))
            print("Std. error", np.std(data_err))
        else:
            print("Success rate is 0")
    
    def cleanup(self):
        """Clean up resources"""
        self.simulator.cleanup()


@hydra.main(version_base=None, config_path="../../conf", config_name="config_xarm6_push")
def run_xarm6_robot(cfg: ExampleConfig):
    """Main function to run the robot simulation"""
    import argparse
    import sys
    
    # Parse command line arguments BEFORE Hydra processes them
    parser = argparse.ArgumentParser(description='MPPI Simulator Interface')
    parser.add_argument('--simulator', type=str, default='isaac_gym', 
                       choices=['isaac_gym', 'gazebo', 'real_robot'],
                       help='Simulator type to use')
    
    # Parse only known args to avoid conflicts with Hydra
    args, unknown = parser.parse_known_args()
    
    # Convert config to object
    cfg = OmegaConf.to_object(cfg)
    
    # Create simulator based on argument
    if args.simulator == 'isaac_gym':
        simulator = IsaacGymSimulator()
    elif args.simulator == 'gazebo':
        print("Gazebo simulator not implemented in current version")
        print("Available simulators: isaac_gym")
        sys.exit(1)
    elif args.simulator == 'real_robot':
        print("Real robot simulator not implemented in current version")
        print("Available simulators: isaac_gym")
        sys.exit(1)
    else:
        simulator = IsaacGymSimulator()
    
    # Create controller
    controller = SimulatorController(simulator)
    
    try:
        # Initialize and run
        controller.initialize(cfg)
        controller.run_control_loop()
        controller.print_results()
    finally:
        controller.cleanup()
    
    return {}


if __name__ == "__main__":
    res = run_xarm6_robot()