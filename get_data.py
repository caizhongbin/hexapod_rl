from actor_model.utils import IK

import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data

import time

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

my_pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
my_pybullet_client.setGravity(0,0,-9.8)

init_position = [0,0,0.31]

urdf_root=pybullet_data.getDataPath()
floor = my_pybullet_client.loadURDF("%s/plane.urdf" % urdf_root)

hexapod_urdf_root = "urdf"

my_hexapod = my_pybullet_client.loadURDF(
                "%s/hexapod.urdf" % hexapod_urdf_root,
                init_position,
                flags=my_pybullet_client.URDF_USE_SELF_COLLISION)


num_joints = my_pybullet_client.getNumJoints(my_hexapod)
joint_name_to_id = {}
for i in range(num_joints):
    joint_info = my_pybullet_client.getJointInfo(my_hexapod, i)
    joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]
motor_id_list = [joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]
joint_leg_joint_id = [[0,1,2,3],[5,6,7,8],[10,11,12,13],[15,16,17,18],[20,21,22,23],[25,26,27,28]]

for i in motor_id_list:
  #disable default constraint-based motors
    my_pybullet_client.setJointMotorControl2(my_hexapod, i, my_pybullet_client.POSITION_CONTROL, targetPosition=0, force=30)

hexapod_ik_utils = IK()
traj_data = hexapod_ik_utils.generate_traj(3, [0.15, 0, 0.3], T_m=2, period_num=1)
joint_data = hexapod_ik_utils.ik_calculate(traj_data)

count = joint_data.shape[2]

useRealTimeSim = True
# timeStep = 0.01
my_pybullet_client.setRealTimeSimulation(useRealTimeSim)

while (1):
    # my_pybullet_client.stepSimulation()
    
    for k in range(count):
        # my_pybullet_client.stepSimulation()
        for j in range(4):
            for i in range(6):
                my_pybullet_client.setJointMotorControl2(my_hexapod, joint_leg_joint_id[i][j], my_pybullet_client.POSITION_CONTROL, targetPosition=joint_data[i][j][k], force=30)
                time.sleep(0.0001) 
        # my_pybullet_client.stepSimulation()        #



