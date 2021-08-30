import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import copy
import math

import numpy as np

INIT_POSITION = [0, 0, 0.31]
INIT_ORIENTATION = [0, 0, 0, 1]
LEG_POSITION = ["leg1", "leg2", "leg3", "leg4", "leg5", "leg6"]
MOTOR_NAMES = [
    "leg1_joint_roll", "leg1_joint_pitch_1", "leg1_joint_pitch_2", "leg1_joint_pitch_3",
    "leg2_joint_roll", "leg2_joint_pitch_1", "leg2_joint_pitch_2", "leg2_joint_pitch_3",
    "leg3_joint_roll", "leg3_joint_pitch_1", "leg3_joint_pitch_2", "leg3_joint_pitch_3",
    "leg4_joint_roll", "leg4_joint_pitch_1", "leg4_joint_pitch_2", "leg4_joint_pitch_3",
    "leg5_joint_roll", "leg5_joint_pitch_1", "leg5_joint_pitch_2", "leg5_joint_pitch_3",
    "leg6_joint_roll", "leg6_joint_pitch_1", "leg6_joint_pitch_2", "leg6_joint_pitch_3"
]
LINK_NAMES = [
    "leg1_roll", "leg1_pitch_1", "leg1_pitch_2", "leg1_pitch_3",
    "leg2_roll", "leg2_pitch_1", "leg2_pitch_2", "leg2_pitch_3",
    "leg3_roll", "leg3_pitch_1", "leg3_pitch_2", "leg3_pitch_3",
    "leg4_roll", "leg4_pitch_1", "leg4_pitch_2", "leg4_pitch_3",
    "leg5_roll", "leg5_pitch_1", "leg5_pitch_2", "leg5_pitch_3",
    "leg6_roll", "leg6_pitch_1", "leg6_pitch_2", "leg6_pitch_3",
]

class Hexapod(object):
    def __init__(self,
               pybullet_client,
               urdf_root="",
               cmd_vel = [1, 0, 0],
               ):
        self.num_motors = 24
        self.num_legs = int(self.num_motors / 4)
        self._pybullet_client = pybullet_client      
        self._urdf_root = urdf_root
        self.time_step = 0.01
        self._motor_velocity_limit = 3
        #self._observation_history = collections.deque(maxlen=100)
        self._action_history = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        self._cmd_vel = cmd_vel
        self._step_counter = 0
        
        self.joint_leg_joint_id = [[0,1,2,3], [5,6,7,8], [10,11,12,13], [15,16,17,18], [20,21,22,23], [25,26,27,28]]
        
        self._joint_history_t1 = np.zeros((1,24))
        self._joint_history_t2 = np.zeros((1,24))
        self._joint_position   = np.zeros((1,24))
        self.Reset()
    
    def Step(self, action):
        self.ApplyAction(action)
        self._pybullet_client.stepSimulation()
        self.ReceiveObservation()
        self._step_counter += 1
 
    def Terminate(self):
        pass

    def _BuildJointNameToIdDict(self):
        """
    self._joint_name_to_id的内容：
    {'leg1_joint_roll': 0, 'leg1_joint_pitch_1': 1, 'leg1_joint_pitch_2': 2, 'leg1_joint_pitch_3': 3, 'leg1_joint_stick': 4, 
     'leg2_joint_roll': 5, 'leg2_joint_pitch_1': 6, 'leg2_joint_pitch_2': 7, 'leg2_joint_pitch_3': 8, 'leg2_joint_stick': 9, 
     'leg3_joint_roll': 10, 'leg3_joint_pitch_1': 11, 'leg3_joint_pitch_2': 12, 'leg3_joint_pitch_3': 13, 'leg3_joint_stick': 14,
     'leg4_joint_roll': 15, 'leg4_joint_pitch_1': 16, 'leg4_joint_pitch_2': 17, 'leg4_joint_pitch_3': 18, 'leg4_joint_stick': 19, 
     'leg5_joint_roll': 20, 'leg5_joint_pitch_1': 21, 'leg5_joint_pitch_2': 22, 'leg5_joint_pitch_3': 23, 'leg5_joint_stick': 24, 
     'leg6_joint_roll': 25, 'leg6_joint_pitch_1': 26, 'leg6_joint_pitch_2': 27, 'leg6_joint_pitch_3': 28, 'leg6_joint_stick': 29}
        """    
        num_joints = self._pybullet_client.getNumJoints(self.my_hexapod)
        self._joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.my_hexapod, i)
            self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]  #[0]是jointindex， [1]jointname
            
    def _BuildMotorIdList(self):
        #[0, 1, 2, 3, 5, 6, 7, 8, 10, 11, 12, 13, 15, 16, 17, 18, 20, 21, 22, 23, 25, 26, 27, 28]
        self._motor_id_list = [self._joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]

    def _BuildLinkNameToIdDict(self):
        num_joints = self._pybullet_client.getNumJoints(self.my_hexapod)
        self._link_name_to_id = {}
        for i in range(num_joints):
            link_info = self._pybullet_client.getJointInfo(self.my_hexapod, i)
            self._link_name_to_id[link_info[12].decode("UTF-8")] = link_info[0]

    def GetTrueObservation(self):
        observation = []
        observation.extend(self.GetBasePosition())            #机体位置向量
        observation.extend(self.GetBaseOrientation())     #机体方向向量
        observation.extend(self.GetTrueBodyLinearVelocity())  #body线速度
        observation.extend(self.GetTrueBodyAngularVelocity())  #body角速度
        #observation.extend(self.GetBaseHigh())  #body高度
        observation.extend(self.GetTrueMotorAngles())         #关节角度
        # observation.extend(self.GetTrueMotorVelocities())     #关节速度
        #observation.extend(self.GetMotorAnglesHistoryT1())#关节历史位置状态信息t-0.01
        #observation.extend(self.GetMotorAnglesHistoryT2())#关节历史位置状态信息t-0.02
        #observation.extend(self._action_history)
        #observation.extend(self._cmd_vel)                     #速度命令
             #腿相位
        return observation

    def ReceiveObservation(self):
        """Receive the observation from sensors.

        This function is called once per step. The observations are only updated
        when this function is called.
        """
        #self._observation_history.appendleft(self.GetTrueObservation())
        self._control_observation = self.GetTrueObservation()

    def Reset(self, reload_urdf=True):
        init_position = INIT_POSITION
        if reload_urdf:
            self.my_hexapod = self._pybullet_client.loadURDF(
                "%s/new_hexapod.urdf" % self._urdf_root,
                init_position,
                flags=self._pybullet_client.URDF_USE_SELF_COLLISION)

            self._BuildJointNameToIdDict()
            self._BuildMotorIdList()
            self.change_dynamics()
            self.ResetPose()
        
        else:
            self._pybullet_client.resetBasePositionAndOrientation(self.my_hexapod, init_position,
                                                                  INIT_ORIENTATION)
            self._pybullet_client.resetBaseVelocity(self.my_hexapod, [0, 0, 0], [0, 0, 0])   
            self.ResetPose() 
        self._step_counter = 0
        self._joint_position = [
            self._pybullet_client.getJointState(self.my_hexapod, motor_id)[0]
            for motor_id in self._motor_id_list
        ]
        self._joint_history_t1 = self._joint_position
        self._joint_history_t2 = self._joint_position
        #self._observation_history.clear()


    def ResetPose(self):
        """Reset the pose of the hexapod.
        """
        for i in range(self.num_legs):
            self._ResetPoseForLeg(i)

    def _ResetPoseForLeg(self, leg_id):
        """Reset the initial pose for the leg.

        Args:
            leg_id: It should be 0, 1, 2, 3, 4, 5, 6....
        """
        self._pybullet_client.setJointMotorControlArray(self.my_hexapod,
              jointIndices=self.joint_leg_joint_id[leg_id],
              controlMode=self._pybullet_client.POSITION_CONTROL,
              targetPositions=[0, 0, 0, 0],
              forces=[30, 30, 30, 30],
              )
        # self._pybullet_client.setJointMotorControl2(
        #   bodyIndex=self.my_hexapod,
        #   jointIndex=self.joint_leg_joint_id[leg_id][0],
        #   controlMode=self._pybullet_client.POSITION_CONTROL,
        #   targetPosition=0,
        #   force=30)
        # self._pybullet_client.setJointMotorControl2(
        #   bodyIndex=self.my_hexapod,
        #   jointIndex=self.joint_leg_joint_id[leg_id][1],
        #   controlMode=self._pybullet_client.POSITION_CONTROL,
        #   targetPosition=0,
        #   force=30)
        # self._pybullet_client.setJointMotorControl2(
        #   bodyIndex=self.my_hexapod,
        #   jointIndex=self.joint_leg_joint_id[leg_id][2],
        #   controlMode=self._pybullet_client.POSITION_CONTROL,
        #   targetPosition=0,
        #   force=30)
        # self._pybullet_client.setJointMotorControl2(
        #   bodyIndex=self.my_hexapod,
        #   jointIndex=self.joint_leg_joint_id[leg_id][3],
        #   controlMode=self._pybullet_client.POSITION_CONTROL,
        #   targetPosition=0,
        #   force=30)
    
    def change_dynamics(self):
        self._pybullet_client.changeDynamics(self.my_hexapod, 4, lateralFriction=4.5, frictionAnchor=1)
        self._pybullet_client.changeDynamics(self.my_hexapod, 9, lateralFriction=4.5, frictionAnchor=1)
        self._pybullet_client.changeDynamics(self.my_hexapod, 14, lateralFriction=4.5, frictionAnchor=1)
        self._pybullet_client.changeDynamics(self.my_hexapod, 19, lateralFriction=4.5, frictionAnchor=1)
        self._pybullet_client.changeDynamics(self.my_hexapod, 24, lateralFriction=4.5, frictionAnchor=1)
        self._pybullet_client.changeDynamics(self.my_hexapod, 29, lateralFriction=4.5, frictionAnchor=1)

    def reset_action(self, actions):
        self._action_history = actions

    def ConvertActionToLegAngle(self, actions):
        """
        关节角度action∈[-1, 1]转换成实际关节角度[-pi/2, pi/2]
        """
        joint_angle = copy.deepcopy(actions)
        half_pi = math.pi/2
        for i in range(self.num_motors):
            joint_angle[i] = actions[i] * half_pi
        return joint_angle

    def GetBaseHigh(self):
        return self.GetBasePosition()[2]

    def GetBaseOrientation(self):
        """Get the position of hexapod's base.
        """
        _, orientation = (self._pybullet_client.getBasePositionAndOrientation(self.my_hexapod))
        return orientation

    def GetBasePosition(self):
        """Get the position of hexapod's base.
        """
        position, _ = (self._pybullet_client.getBasePositionAndOrientation(self.my_hexapod))
        return position

    def GetBasePositionAndOrientation(self):
        """Get the position of body.
                                  [x,y,z,w]
            ((0.0, 0.0, 0.31), (0.0, 0.0, 0.0, 1.0))
        """
        position, orientation = (self._pybullet_client.getBasePositionAndOrientation(self.my_hexapod))
        return position, orientation
    
    def GetTrueMotorAngles(self):
        """Gets the motor angles at the current moment
        Returns:
            Motor angles
        """
        motor_angles = [self._pybullet_client.getJointState(self.my_hexapod, motor_id)[0] for motor_id in self._motor_id_list]
        motor_angles = np.array(motor_angles)
        self._joint_history_t2 = self._joint_history_t1 
        self._joint_history_t1 = self._joint_position
        self._joint_position = motor_angles

        return motor_angles
    
    def GetMotorAnglesHistoryT1(self):
        return self._joint_history_t1

    def GetMotorAnglesHistoryT2(self):
        return self._joint_history_t2

    def GetTrueMotorVelocities(self):
        """Get the velocity of all motors.

        Returns:
            Velocities of all motors.
        """
        motor_velocities = [
            self._pybullet_client.getJointState(self.my_hexapod, motor_id)[1]
            for motor_id in self._motor_id_list
        ]
        #motor_velocities = np.multiply(motor_velocities, self._motor_direction)
        return motor_velocities

    def GetTrueMotorTorques(self):
        """Get the amount of torque the motors are exerting.

        Returns:
        Motor torques of all motors.
        """
        motor_torques = [
            self._pybullet_client.getJointState(self.my_hexapod, motor_id)[3]
            for motor_id in self._motor_id_list
        ]
        return motor_torques
    
    def GetTrueBodyLinearVelocity(self):
        vel, _ = self._pybullet_client.getBaseVelocity(self.my_hexapod)
        return vel

    def GetTrueBodyAngularVelocity(self):
        _, yaw = self._pybullet_client.getBaseVelocity(self.my_hexapod)
        return yaw
    
    def GetBodyVelocity(self):
        vel_and_ang = self._pybullet_client.getBaseVelocity(self.my_hexapod)
        return vel_and_ang


    def ApplyAction(self, motor_commands):
        current_joint_angles = self.GetTrueMotorAngles()
        motor_commands_max = (current_joint_angles + self.time_step * self._motor_velocity_limit)
        motor_commands_min = (current_joint_angles - self.time_step * self._motor_velocity_limit)
        motor_commands = np.clip(motor_commands, motor_commands_min, motor_commands_max)
        for i in range(self.num_legs):
            self._pybullet_client.setJointMotorControlArray(self.my_hexapod,
              jointIndices=self.joint_leg_joint_id[i],
              controlMode=self._pybullet_client.POSITION_CONTROL,
              targetPositions=[motor_commands[i*4], motor_commands[i*4+1], motor_commands[i*4+2], motor_commands[i*4+3]],
              forces=[30, 30, 30, 30],
              positionGains=[1, 1, 1, 1],
              velocityGains=[1, 1, 1, 1]
              )
        