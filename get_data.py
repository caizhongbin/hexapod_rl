from actor_model.utils import IK

import pybullet
from pybullet_utils import bullet_client as bc
import pybullet_data

import time
import numpy as np
import matplotlib.pyplot as plt

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

def calculateMSE(x, y):
    return sum((a-b)**2 for a, b in zip(x, y))/len(x)

def initPybullet(): 
    my_pybullet_client = bc.BulletClient(connection_mode=pybullet.GUI)
    my_pybullet_client.setGravity(0,0,-9.8)
    my_pybullet_client.configureDebugVisualizer(my_pybullet_client.COV_ENABLE_WIREFRAME, 0)
    my_pybullet_client.configureDebugVisualizer(my_pybullet_client.COV_ENABLE_GUI, 0) 
    urdf_root=pybullet_data.getDataPath()
    floor = my_pybullet_client.loadURDF("%s/plane.urdf" %urdf_root)
    return my_pybullet_client

def loadRobotModel(my_pybullet_client, hexapod_urdf_root):
    init_position = [0,0,0.31]
    my_hexapod = my_pybullet_client.loadURDF(
                "%s/new_hexapod.urdf" %hexapod_urdf_root,
                init_position,
                flags=my_pybullet_client.URDF_USE_SELF_COLLISION)
    return my_hexapod

def getTraj():
    hexapod_ik_utils = IK()
    traj_data = hexapod_ik_utils.cubic_bezier_traj(3, [0.1, 0, 0.15], T_m=2, period_num=2)
    joint_data = hexapod_ik_utils.ik_calculate(traj_data)
    return joint_data

if __name__ == '__main__':
    kp = 1.2
    kd = 1
    my_pybullet_client = initPybullet()
    my_hexapod = loadRobotModel(my_pybullet_client, "urdf")

    num_joints = my_pybullet_client.getNumJoints(my_hexapod)
    joint_name_to_id = {}

    for i in range(num_joints):
        joint_info = my_pybullet_client.getJointInfo(my_hexapod, i)
        joint_name_to_id[joint_info[1].decode("UTF-8")] = joint_info[0]

    motor_id_list = [joint_name_to_id[motor_name] for motor_name in MOTOR_NAMES]
    joint_leg_joint_id = [[0,1,2,3],[5,6,7,8],[10,11,12,13],[15,16,17,18],[20,21,22,23],[25,26,27,28]]
    
    for i in range(num_joints):
        joint_info = my_pybullet_client.getJointInfo(my_hexapod, i)
        my_pybullet_client.changeDynamics(joint_info[0], -1, linearDamping=0, angularDamping=0)
    
    for i in motor_id_list:
        #disable default constraint-based motors
        my_pybullet_client.setJointMotorControl2(my_hexapod, i, my_pybullet_client.POSITION_CONTROL, targetPosition=0, force=30)
    
    joint_data = getTraj()
    count = joint_data.shape[2]
    real_joint_data = np.zeros(count)
    joint_torque = np.zeros((6, 4, count))

    pre_pos = my_pybullet_client.getLinkState(my_hexapod, 4)[0]

    time_con = 0.01
    last_time = time.time()
    first_position, first_orientation = my_pybullet_client.getBasePositionAndOrientation(my_hexapod)
    for _ in range(1):
    # my_pybullet_client.stepSimulation()
        for k in range(count):
        # my_pybullet_client.stepSimulation()
            target_pos = my_pybullet_client.getLinkState(my_hexapod, 4)[0]
            my_pybullet_client.addUserDebugLine(pre_pos, target_pos, lineColorRGB=[1, 0, 0], lifeTime=8, lineWidth=3)
        # for j in range(4):
            for i in range(6):
                # my_pybullet_client.setJointMotorControl2(my_hexapod, 
                #                                          joint_leg_joint_id[i][j], 
                #                                          my_pybullet_client.POSITION_CONTROL, 
                #                                          targetPosition=joint_data[i][j][k],
                #                                          positionGain=kp,
                #                                          velocityGain=kd, 
                #                                          force=30)
                my_pybullet_client.setJointMotorControlArray(my_hexapod,
                    jointIndices=joint_leg_joint_id[i],
                    controlMode=my_pybullet_client.POSITION_CONTROL,
                    targetPositions=[joint_data[i][0][k], joint_data[i][1][k], joint_data[i][2][k], joint_data[i][3][k]],
                    forces=[30, 30, 30, 30],
                    positionGains=[kp, kp, kp, kp],
                    velocityGains=[kd, kd, kd, kd]
                )
                # my_pybullet_client.setJointMotorControl2(my_hexapod, 
                #                                          joint_leg_joint_id[i][j], 
                #                                          my_pybullet_client.POSITION_CONTROL, 
                #                                          targetPosition=joint_data[i][j][k],
                #                                          force=30)
                # time.sleep(0.0001)
        
            time_spend = time.time()-last_time
            last_time = time.time()
            time_to_sleep = time_con-time_spend
            if time_to_sleep>0:
                time.sleep(time_to_sleep)
            my_pybullet_client.stepSimulation()
        
            for j in range(4):
                for i in range(6):
                    joint_torque[i][j][k] = my_pybullet_client.getJointState(my_hexapod, joint_leg_joint_id[i][j])[3]
            # print(my_pybullet_client.getBaseVelocity(my_hexapod))
            real_joint_data[k] = my_pybullet_client.getJointState(my_hexapod, 0)[0]
        
            pre_pos = target_pos
        # my_pybullet_client.stepSimulation() 

    last_position, last_orientation = my_pybullet_client.getBasePositionAndOrientation(my_hexapod)
    
    fig = plt.figure()
    ax = fig.add_subplot(1, 1, 1)
    ax.plot(joint_torque[0][0][:])
    plt.show()

# my_hexapod = my_pybullet_client.loadURDF(
#                 "%s/hexapod.urdf" % hexapod_urdf_root,
#                 init_position,
#                 flags=my_pybullet_client.URDF_USE_SELF_COLLISION)

# my_pybullet_client.changeDynamics(my_hexapod, 4, lateralFriction=0.6, spinningFriction=0.2, rollingFriction=0.5)
# my_pybullet_client.changeDynamics(my_hexapod, 9, lateralFriction=0.6, spinningFriction=0.2, rollingFriction=0.5)
# my_pybullet_client.changeDynamics(my_hexapod, 14, lateralFriction=0.6, spinningFriction=0.2, rollingFriction=0.5)
# my_pybullet_client.changeDynamics(my_hexapod, 19, lateralFriction=0.6, spinningFriction=0.2, rollingFriction=0.5)
# my_pybullet_client.changeDynamics(my_hexapod, 24, lateralFriction=0.6, spinningFriction=0.2, rollingFriction=0.5)
# my_pybullet_client.changeDynamics(my_hexapod, 29, lateralFriction=0.6, spinningFriction=0.2, rollingFriction=0.5)
#print(my_pybullet_client.getDynamicsInfo(my_hexapod, 4))
# my_pybullet_client.changeDynamics(my_hexapod, 4, lateralFriction=4.5, frictionAnchor=1)
# my_pybullet_client.changeDynamics(my_hexapod, 9, lateralFriction=4.5, frictionAnchor=1)
# my_pybullet_client.changeDynamics(my_hexapod, 14, lateralFriction=4.5, frictionAnchor=1)
# my_pybullet_client.changeDynamics(my_hexapod, 19, lateralFriction=4.5, frictionAnchor=1)
# my_pybullet_client.changeDynamics(my_hexapod, 24, lateralFriction=4.5, frictionAnchor=1)
# my_pybullet_client.changeDynamics(my_hexapod, 29, lateralFriction=4.5, frictionAnchor=1)

# useRealTimeSim = True
# timeStep = 0.01
# my_pybullet_client.setRealTimeSimulation(useRealTimeSim)
# while (1):

# print([last_position[0]-first_position[0], 
#         last_position[1]-first_position[1],
#         last_position[2]-first_position[2]])

# print([last_orientation[0]-first_orientation[0],
#        last_orientation[1]-first_orientation[1],
#        last_orientation[2]-first_orientation[2],
#        last_orientation[3]-first_orientation[3]])

# print(calculate_mse(real_joint_data, joint_data[0][2][:]))
# x = np.linspace(0, 1, count)
# fig, ax = plt.subplots()
# ax.plot(x, real_joint_data, label='real_joint_data label')
# ax.plot(x, joint_data[0][0][:], label='command_joint_data label')
# ax.set_xlabel('time')
# ax.set_ylabel('joint angle')
# ax.set_title('control and real area chart')
# ax.legend()
# plt.show()
