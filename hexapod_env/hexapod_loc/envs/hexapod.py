import os, inspect
currentdir = os.path.dirname(os.path.abspath(inspect.getfile(inspect.currentframe())))
parentdir = os.path.dirname(os.path.dirname(currentdir))
os.sys.path.insert(0, parentdir)

import collections
import copy
import math
import re

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


class Hexapod(object):
    def __init__(self,
               pybullet_client,
               urdf_root="",
            
               on_rack=False):
        self.num_motors = 24
        self.num_legs = int(self.num_motors / 4)
        self._pybullet_client = pybullet_client      
        self._urdf_root = urdf_root
        self.Reset(reset_time=-1.0)


    def Reset(self, reload_urdf=True, default_motor_angles=None, reset_time=3.0):
        
        init_position = INIT_POSITION
        if reload_urdf:
            self.my_hexapod = self._pybullet_client.loadURDF(
                "%s/hexapod.urdf" %self._urdf_root,
                init_position,
                flags=self._pybullet_client.URDF_USE_SELF_COLLISION)

            self._BuildJointNameToIdDict()
            self.ResetPose()
        
        else:
            self._pybullet_client.resetBasePositionAndOrientation(self.my_hexapod, init_position,
                                                            INIT_ORIENTATION)
            self._pybullet_client.resetBaseVelocity(self.my_hexapod, [0, 0, 0], [0, 0, 0])   
            self.ResetPose()      

    def _BuildJointNameToIdDict(self):
        num_joints = self._pybullet_client.getNumJoints(self.my_hexapod)
        self._joint_name_to_id = {}
        for i in range(num_joints):
            joint_info = self._pybullet_client.getJointInfo(self.my_hexapod, i)
            self._joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]  #[0]是jointindex， [1]jointname
            

    def ResetPose(self):
        """Reset the pose of the hexapod.
        """
        for i in range(self.num_legs):
            self._ResetPoseForLeg(i)

    def _ResetPoseForLeg(self, leg_id):
        """Reset the initial pose for the leg.

        Args:
            leg_id: It should be 0, 1, 2, 3, 4, 5, 6
        """
        leg_position = LEG_POSITION[leg_id]
        self._pybullet_client.resetJointState(self.my_hexapod,
                                            self._joint_name_to_id[leg_position + "joint_roll"],
                                            0,
                                            targetVelocity=0)
        self._pybullet_client.resetJointState(self.my_hexapod,
                                            self._joint_name_to_id[leg_position + "joint_pitch_1"],
                                            0,
                                            targetVelocity=0)
        self._pybullet_client.resetJointState(self.my_hexapod,
                                            self._joint_name_to_id[leg_position + "joint_pitch_2"],
                                            0,
                                            targetVelocity=0)
        self._pybullet_client.resetJointState(self.my_hexapod,
                                            self._joint_name_to_id[leg_position + "joint_pitch_3"],
                                            0,
                                            targetVelocity=0)

        self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.my_hexapod,
          jointIndex=(self._joint_name_to_id[leg_position + "joint_roll"]),
          controlMode=self._pybullet_client.POSITION_CONTROL,
          targetPosition=0,
          force=20)
        self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.my_hexapod,
          jointIndex=(self._joint_name_to_id[leg_position + "joint_pitch_1"]),
          controlMode=self._pybullet_client.POSITION_CONTROL,
          targetPosition=0,
          force=20)
        self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.my_hexapod,
          jointIndex=(self._joint_name_to_id[leg_position + "joint_pitch_2"]),
          controlMode=self._pybullet_client.POSITION_CONTROL,
          targetPosition=0,
          force=20)
        self._pybullet_client.setJointMotorControl2(
          bodyIndex=self.my_hexapod,
          jointIndex=(self._joint_name_to_id[leg_position + "joint_pitch_3"]),
          controlMode=self._pybullet_client.POSITION_CONTROL,
          targetPosition=0,
          force=20)
        

