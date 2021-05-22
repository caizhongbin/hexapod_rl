import numpy as np
import math
import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data
import time
from gym import spaces

a = np.zeros(3)
print(a)
a[0]=1
a[1]=2
a[2]=3
print(a)
a = a - np.mean(a)  
a = a / np.max(np.abs(a)) 
print(a)


# MOTOR_NAMES = [
#     "leg1_joint_roll", "leg1_joint_pitch_1", "leg1_joint_pitch_2", "leg1_joint_pitch_3",
#     "leg2_joint_roll", "leg2_joint_pitch_1", "leg2_joint_pitch_2", "leg2_joint_pitch_3",
#     "leg3_joint_roll", "leg3_joint_pitch_1", "leg3_joint_pitch_2", "leg3_joint_pitch_3",
#     "leg4_joint_roll", "leg4_joint_pitch_1", "leg4_joint_pitch_2", "leg4_joint_pitch_3",
#     "leg5_joint_roll", "leg5_joint_pitch_1", "leg5_joint_pitch_2", "leg5_joint_pitch_3",
#     "leg6_joint_roll", "leg6_joint_pitch_1", "leg6_joint_pitch_2", "leg6_joint_pitch_3"
# ]
# my_pybullet_client = bc.BulletClient()

# my_pybullet_client.setGravity(0, 0, -10)
# init_position = [0,0,0.31]

# urdf_root=pybullet_data.getDataPath()
# floor = my_pybullet_client.loadURDF("%s/plane.urdf" % urdf_root)

# hexapod_urdf_root = "urdf"

# my_hexapod = my_pybullet_client.loadURDF(
#                 "%s/hexapod.urdf" % hexapod_urdf_root,
#                 init_position,
#                 flags=my_pybullet_client.URDF_USE_SELF_COLLISION)

# num_joints = my_pybullet_client.getNumJoints(my_hexapod)
# joint_name_to_id = {}
# for i in range(num_joints):
#     joint_info = my_pybullet_client.getJointInfo(my_hexapod, i)
#     joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

# motor_id_list = [joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]

# def GetTrueBodyLinearVelocity():
#     vel, _ = my_pybullet_client.getBaseVelocity(my_hexapod)
#     return vel

# def GetTrueBodyAngularVelocity():
#     _, yaw = my_pybullet_client.getBaseVelocity(my_hexapod)
#     return yaw

# def GetTrueMotorAngles():
#     motor_angles = [my_pybullet_client.getJointState(my_hexapod, motor_id)[0] for motor_id in motor_id_list]
#     return motor_angles

# def GetTrueMotorVelocities():
#     motor_velocities = [
#         my_pybullet_client.getJointState(my_hexapod, motor_id)[1]
#         for motor_id in motor_id_list
#     ]
#     return motor_velocities

# def GetBasePosition():
#     position, _ = (my_pybullet_client.getBasePositionAndOrientation(my_hexapod))
#     return position
# def GetBaseOrientation():
#     _, orientation = (my_pybullet_client.getBasePositionAndOrientation(my_hexapod))
#     return orientation

# observation = []
# observation.extend(list(GetBasePosition()))            #机体位置向量
# observation.extend(list(GetBaseOrientation()))
# observation.extend(list(GetTrueBodyLinearVelocity()))
# observation.extend(list(GetTrueBodyAngularVelocity()))
# observation.extend(GetTrueMotorAngles())         #关节角度
# observation.extend(GetTrueMotorVelocities()) 
# observation = np.array(observation)

# def _get_observation_upper_bound():
#     upper_bound = np.zeros(61)
#     upper_bound[0:3] = 5  # base_position
#     upper_bound[3:7] = 1.0  # base_orientation
#     upper_bound[7:10] = 5.0 #base linear vel
#     upper_bound[10:13] = 5.0 #base angular vel
#     upper_bound[13:37] = math.pi/2 #joint position
#     upper_bound[37:61] = 5 #joint vel
#     return upper_bound

# my_upper_bound = _get_observation_upper_bound()
# print(my_upper_bound)
# s = spaces.Box(-my_upper_bound, my_upper_bound)
# print(s)
# print(s.contains(observation))


# num_joints = my_pybullet_client.getNumJoints(my_hexapod)
# link_name_to_id = {}
# for i in range(num_joints):
#     link_info = my_pybullet_client.getJointInfo(my_hexapod, i)
#     link_name_to_id[link_info[12].decode("UTF-8")] = link_info[0]
# print(link_name_to_id)

# num = my_pybullet_client.getNumJoints(my_hexapod)
# print(num)

# joint_info = my_pybullet_client.getJointInfo(my_hexapod, 0)
# print(joint_info)
# def _BuildJointNameToIdDict():
#     num_joints = my_pybullet_client.getNumJoints(my_hexapod)
#     joint_name_to_id = {}
#     for i in range(num_joints):
#         joint_info = my_pybullet_client.getJointInfo(my_hexapod, i)
#         joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
#     print(joint_name_to_id)
#     motor_id_list = [joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]
#     print(motor_id_list)

# _BuildJointNameToIdDict()
#print(my_pybullet_client.getBasePositionAndOrientation(my_hexapod))



# num_bodys = my_pybullet_client.getNumBodies(my_hexapod)
# num_bodys = my_pybullet_client.getDynamicsInfo(my_hexapod, 0)
# print(num_bodys)
# joint_name_to_id = {}
# for i in range(num_joints):
#     joint_info = my_pybullet_client.getBodyInfo(my_hexapod, i)
#     joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
# print(joint_name_to_id)
        
