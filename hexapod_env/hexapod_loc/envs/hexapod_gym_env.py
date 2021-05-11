import gym
from gym import spaces
from gym.utils import seeding

import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data

import numpy as np
from gym.utils import seeding


class HexapodGymEnv(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 100}
	
    def __init__(self, urdf_root=pybullet_data.getDataPath(), urdf_version=None, render=False, log_path=None):	   
        self._urdf_root = urdf_root
        self._observation = []
        self._is_render = render
        if self._is_render:
            self._pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
        else:
            self._pybullet_client = bc.BulletClient()
        
        self._hard_reset = True
		
        self.seed()
        self.reset()

		
        self.action_space = spaces.Box(-action_high, action_high)
        self.observation_space = spaces.Box(observation_low, observation_high)	 
        
        self._hard_reset = hard_reset
			 
    # def step(self, action):
    #     """Step forward the simulation, given the action.

    #     Args:
    #     action: A list of desired motor angles for eight motors.

    #     Returns:
    #       observations: The angles, velocities and torques of all motors.
    #       reward: The reward for the current state-action pair.
    #       done: Whether the episode has ended.
    #       info: A dictionary that stores diagnostic information.

    #     Raises:
    #       ValueError: The action dimension is not the same as the number of motors.
    #       ValueError: The magnitude of actions is out of bounds.
    #     """
    def reset(self, initial_motor_angles=None, reset_duration=1.0):
	    #重新初始化
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING, 0)
	
        if self._hard_reset:
            self._pybullet_client.resetSimulation()
            self._pybullet_client.setGravity(0, 0, -10)
            self._ground_id = self._pybullet_client.loadURDF("%s/plane.urdf" % self._urdf_root)	
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING, 1)
        return self._get_observation()	
		
    def render(self, mode="rgb_array", close=False):
        return

    def close(self):
        return

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _reward(self):
        my_reward = []
        return my_reward

    def _get_observation(self):
        observation = []
        #observation.extend(self.minitaur.GetMotorAngles().tolist())
        #observation.extend(self.minitaur.GetMotorVelocities().tolist())
        #observation.extend(self.minitaur.GetMotorTorques().tolist())
        #observation.extend(list(self.minitaur.GetBaseOrientation()))
        self._observation = observation
        return self._observation
    
    def _transform_action_to_motor_command(self, action):
        action = self.hexapod.ConvertActionToLegAngle(action)
        return action