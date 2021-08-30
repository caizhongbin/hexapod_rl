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
import time

RENDER_HEIGHT = 360
RENDER_WIDTH = 480
NUM_MOTORS = 24

class HexapodGymEnv(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 100}
	
    def __init__(self, urdf_root=pybullet_data.getDataPath(), 
                 render=False,
                 distance_limit=10,
                 forward_reward_cap=float("inf"), 
                 distance_weight=3.0,
                 energy_weight=0.02,
                 drift_weight=0.0,
                 shake_weight=0.0,
                 hard_reset=True,
                 hexapod_urdf_root="/home/czbfy/hexapod_rl/urdf"):
        super(HexapodGymEnv, self).__init__()  
        self._urdf_root = urdf_root
        self._hexapod_urdf_root = hexapod_urdf_root
        self._observation = []
        self._norm_observation = []
        self._env_step_counter = 0
        self._is_render = render
        self._cam_dist = 1.0
        self._cam_yaw = 0
        self._cam_pitch = -30
        self._last_frame_time = 0.0
        self.control_time_step = 0.01
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

        self._action_bound = 1.0
        action_dim = NUM_MOTORS
        action_high = np.array([self._action_bound] * action_dim)
        self.action_space = spaces.Box(-action_high, action_high)

        observation_high = self._get_observation_upper_bound()
        observation_low = -observation_high
        self.observation_space = spaces.Box(observation_low, observation_high)	 
        self._hard_reset = hard_reset

    def step(self, action):
        """Step forward the simulation, given the action.

        Args:
        action: A list of desired motor angles for motors.

        Returns:
          observations: 
          reward: The reward for the current state-action pair.
          done: Whether the episode has ended.
          info: A dictionary that stores diagnostic information.
        """
        self._last_base_position = self.hexapod.GetBasePosition()
        
        time_spent = time.time() - self._last_frame_time
        self._last_frame_time = time.time()
        time_to_sleep = self.control_time_step - time_spent
        if time_to_sleep > 0:
            time.sleep(time_to_sleep)

        action = self._transform_action_to_motor_command(action)
        self.hexapod.Step(action)
        reward = self._reward()
        done = self._termination()

        self._env_step_counter += 1

        if done:
            self.hexapod.Terminate()
        
        observation = np.array(self._get_observation()).astype(np.float32)
        info = {}
        return observation, reward, done, info

    def reset(self):
	    #重新初始化
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING, 0)
	
        if self._hard_reset:
            self._pybullet_client.resetSimulation()
            self._pybullet_client.setGravity(0, 0, -10)
            self._ground_id = self._pybullet_client.loadURDF("%s/plane.urdf" % self._urdf_root)
            self.hexapod = Hexapod(pybullet_client=self._pybullet_client, urdf_root=self._hexapod_urdf_root)
        
        self.hexapod.Reset(reload_urdf=False)
        self._env_step_counter = 0
        self._last_base_position = [0, 0, 0]
        self._pybullet_client.resetDebugVisualizerCamera(self._cam_dist, self._cam_yaw,
                                                     self._cam_pitch, [0, 0, 0])
        self._pybullet_client.configureDebugVisualizer(self._pybullet_client.COV_ENABLE_RENDERING, 1)
        return np.array(self._get_observation())
		
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
        self.hexapod.Terminate()

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
        observation.extend(list(self.hexapod.GetBasePosition()))
        observation.extend(list(self.hexapod.GetBaseOrientation()))
        observation.extend(list(self.hexapod.GetTrueBodyLinearVelocity()))
        observation.extend(list(self.hexapod.GetTrueBodyAngularVelocity()))
        observation.extend(self.hexapod.GetTrueMotorAngles())
        # observation.extend(self.hexapod.GetTrueMotorVelocities())
        # observation = observation - np.mean(observation)  
        # observation = observation / np.max(np.abs(observation)) 
        self._observation = observation
        return self._observation
    

    def _transform_action_to_motor_command(self, action):
        action = self.hexapod.ConvertActionToLegAngle(action)
        return action
    
    def _termination(self):
        position = self.hexapod.GetBasePosition()
        distance = math.sqrt(position[0]**2 + position[1]**2)
        return self.is_fallen() or position[2]>0.35 or distance > self._distance_limit

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
        upper_bound = np.zeros(37)
        upper_bound[0:3] = self._distance_limit  # base_position
        upper_bound[3:7] = 1.0  # base_orientation
        upper_bound[7:10] = 3.0 #base linear vel
        upper_bound[10:13] = 3.0 #base angular vel
        upper_bound[13:37] = math.pi/2 #joint position
        # upper_bound[37:61] = 3.0 #joint vel
        # upper_bound[num_motors:2 * num_motors] = (motor.MOTOR_SPEED_LIMIT)  # Joint velocity.
        # upper_bound[2 * num_motors:3 * num_motors] = (motor.OBSERVED_TORQUE_LIMIT)  # Joint torque.
        # upper_bound[3 * num_motors:] = 1.0  # Quaternion of base orientation.
        return upper_bound

    def _get_observation_lower_bound(self):
        return -self._get_observation_upper_bound()

    def _get_observation_dimension(self):
        return len(self._get_observation())