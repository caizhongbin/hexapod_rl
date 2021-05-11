import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data
import time
MOTOR_NAMES = [
    "leg1_joint_roll", "leg1_joint_pitch_1", "leg1_joint_pitch_2", "leg1_joint_pitch_3",
    "leg2_joint_roll", "leg2_joint_pitch_1", "leg2_joint_pitch_2", "leg2_joint_pitch_3",
    "leg3_joint_roll", "leg3_joint_pitch_1", "leg3_joint_pitch_2", "leg3_joint_pitch_3",
    "leg4_joint_roll", "leg4_joint_pitch_1", "leg4_joint_pitch_2", "leg4_joint_pitch_3",
    "leg5_joint_roll", "leg5_joint_pitch_1", "leg5_joint_pitch_2", "leg5_joint_pitch_3",
    "leg6_joint_roll", "leg6_joint_pitch_1", "leg6_joint_pitch_2", "leg6_joint_pitch_3"
]
my_pybullet_client = bc.BulletClient()

my_pybullet_client.setGravity(0, 0, -10)
init_position = [0,0,0.31]

urdf_root=pybullet_data.getDataPath()
floor = my_pybullet_client.loadURDF("%s/plane.urdf" % urdf_root)

hexapod_urdf_root = "urdf"

my_hexapod = my_pybullet_client.loadURDF(
                "%s/hexapod.urdf" % hexapod_urdf_root,
                init_position,
                flags=my_pybullet_client.URDF_USE_SELF_COLLISION)
        
num_joints = my_pybullet_client.getNumJoints(my_hexapod)
link_name_to_id = {}
for i in range(num_joints):
    link_info = my_pybullet_client.getJointInfo(my_hexapod, i)
    link_name_to_id[link_info[12].decode("UTF-8")] = link_info[0]
print(link_name_to_id)

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
        
