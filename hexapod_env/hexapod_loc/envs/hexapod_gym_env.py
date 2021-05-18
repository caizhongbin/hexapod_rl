import gym
from gym import spaces
from gym.utils import seeding

import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data

import numpy as np
from gym.utils import seeding
from hexapod_env.hexapod_loc.envs.hexapod import Hexapod
import math

RENDER_HEIGHT = 360
RENDER_WIDTH = 480
NUM_MOTORS = 24

class HexapodGymEnv(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 100}
	
    def __init__(self, urdf_root=pybullet_data.getDataPath(), 
                 render=False,
                 distance_limit=float("inf"),
                 forward_reward_cap=float("inf"), 
                 distance_weight=1.0,
                 energy_weight=0.005,
                 drift_weight=0.0,
                 shake_weight=0.0,
                 hard_reset=True):	   
        self._urdf_root = urdf_root
        self._observation = []
        self._is_render = render
        self._cam_dist = 1.0
        self._cam_yaw = 0
        self._cam_pitch = -30
        self._distance_limit = distance_limit
        self._forward_reward_cap = forward_reward_cap
        self._time_step = 0.01
        self._objective_weights = [distance_weight, energy_weight, drift_weight, shake_weight]
        self._objectives = []
        if self._is_render:
            self._pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
        else:
            self._pybullet_client = bc.BulletClient()
        
        self._hard_reset = True
		
        self.seed()
        self.reset()

        self._action_bound = 1
        action_dim = NUM_MOTORS
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)
        observation_low = -1
        observation_high = 1
        self.observation_space = spaces.Box(observation_low, observation_high)	 
        
        self._hard_reset = hard_reset
			 
    def step(self, action):
        """Step forward the simulation, given the action.

        Args:
        action: A list of desired motor angles for eight motors.

        Returns:
          observations: The angles, velocities and torques of all motors.
          reward: The reward for the current state-action pair.
          done: Whether the episode has ended.
          info: A dictionary that stores diagnostic information.

        Raises:
          ValueError: The action dimension is not the same as the number of motors.
          ValueError: The magnitude of actions is out of bounds.
        """
        self._last_base_position = self.hexapod.GetBasePosition()

        action = self._transform_action_to_motor_command(action)
        self.hexapod.Step(action)
        reward = self._reward()
        done = self._termination()

        if done:
            self.hexapod.Terminate()

        return np.array(self._get_observation()), reward, done, {}

    def reset(self, initial_motor_angles=None, reset_duration=1.0):
	    #重新初始化
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING, 0)
	
        if self._hard_reset:
            self._pybullet_client.resetSimulation()
            self._pybullet_client.setGravity(0, 0, -10)
            self._ground_id = self._pybullet_client.loadURDF("%s/plane.urdf" % self._urdf_root)
            self.hexapod = Hexapod()
        
        self.hexapod.Reset(reload_urdf=False)
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING, 1)
        return self._get_observation()	
		
    def render(self, mode="rgb_array", close=False):
        if mode != "rgb_array":
            return np.array([])
        base_pos = self.hexapod.GetBasePosition()
        view_matrix = self._pybullet_client.computeViewMatrixFromYawPitchRoll(
            cameraTargetPosition=base_pos,
            distance=self._cam_dist,
            yaw=self._cam_yaw,
            pitch=self._cam_pitch,
            roll=0,
            upAxisIndex=2)
        proj_matrix = self._pybullet_client.computeProjectionMatrixFOV(fov=60,
                                                                       aspect=float(RENDER_WIDTH) /
                                                                       RENDER_HEIGHT,
                                                                       nearVal=0.1,
                                                                       farVal=100.0)
        (_, _, px, _, _) = self._pybullet_client.getCameraImage(
            width=RENDER_WIDTH,
            height=RENDER_HEIGHT,
            renderer=self._pybullet_client.ER_BULLET_HARDWARE_OPENGL,
            viewMatrix=view_matrix,
            projectionMatrix=proj_matrix)
        rgb_array = np.array(px)
        rgb_array = rgb_array[:, :, :3]
        return rgb_array

    def close(self):
        return

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _reward(self):
        current_base_position = self.hexapod.GetBasePosition()
        forward_reward = current_base_position[0] - self._last_base_position[0]
    # Cap the forward reward if a cap is set.
        forward_reward = min(forward_reward, self._forward_reward_cap)
    # Penalty for sideways translation.
        drift_reward = -abs(current_base_position[1] - self._last_base_position[1])
    # Penalty for sideways rotation of the body.
        orientation = self.hexapod.GetBaseOrientation()
        rot_matrix = pybullet.getMatrixFromQuaternion(orientation)
        local_up_vec = rot_matrix[6:]
        shake_reward = -abs(np.dot(np.asarray([1, 1, 0]), np.asarray(local_up_vec)))
        energy_reward = -np.abs(
            np.dot(self.hexapod.GetTrueMotorTorques(),
                   self.hexapod.GetTrueMotorVelocities())) * self._time_step
        objectives = [forward_reward, energy_reward, drift_reward, shake_reward]
        weighted_objectives = [o * w for o, w in zip(objectives, self._objective_weights)]
        reward = sum(weighted_objectives)
        self._objectives.append(objectives)
        return reward
    
    def get_objectives(self):
        return self._objectives
    
    @property
    def objective_weights(self):
        return self._objective_weights

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
    
    def _termination(self):
        position = self.minitaur.GetBasePosition()
        distance = math.sqrt(position[0]**2 + position[1]**2)
        return self.is_fallen() or distance > self._distance_limit

    def is_fallen(self):
        """Decide whether the hexapod has fallen.

        If the up directions between the base and the world is larger (the dot
        product is smaller than 0.85) or the base is very low on the ground
        (the height is smaller than 0.13 meter), the hexapod is considered fallen.

        Returns:
            Boolean value that indicates whether the hexapod has fallen.
        """
        orientation = self.hexapod.GetBaseOrientation()
        rot_mat = self._pybullet_client.getMatrixFromQuaternion(orientation)
        local_up = rot_mat[6:]
        pos = self.hexapod.GetBasePosition()
        return (np.dot(np.asarray([0, 0, 1]), np.asarray(local_up)) < 0.85 or pos[2] < 0.13)

    def _get_observation_upper_bound(self):
        """Get the upper bound of the observation.
        """
        upper_bound = np.zeros(self._get_observation_dimension())
        # num_motors = self.minitaur.num_motors
        # upper_bound[0:num_motors] = math.pi  # Joint angle.
        # upper_bound[num_motors:2 * num_motors] = (motor.MOTOR_SPEED_LIMIT)  # Joint velocity.
        # upper_bound[2 * num_motors:3 * num_motors] = (motor.OBSERVED_TORQUE_LIMIT)  # Joint torque.
        # upper_bound[3 * num_motors:] = 1.0  # Quaternion of base orientation.
        return upper_bound

    def _get_observation_lower_bound(self):
        return -self._get_observation_upper_bound()

    def _get_observation_dimension(self):
        return len(self._get_observation())