import pybullet as p
import pybullet_data

import time
import pathlib

dt = 1/240

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

p.loadURDF('plane.urdf')

cubeStartPos = [0, 0, 1.13]
cubeStartOrientation = p.getQuaternionFromEuler([0., 0, 0])
urdf_file = pathlib.Path("./URDF/cyberpod.urdf")
robot_id = p.loadURDF(urdf_file.absolute().as_posix(), cubeStartPos, 
    cubeStartOrientation)

p.setGravity(0, 0, -9.807)
p.setTimeStep(dt)

joint_name2id = {}
for i in range(p.getNumJoints(robot_id)):
    joint_info = p.getJointInfo(robot_id, i)
    joint_name2id[joint_info[1].decode('UTF-8')] = joint_info[0]

# non-torque control
target_angle = 0 # for position control
target_speed = 1 # for velocity control
max_force = 1

# enable torque control
max_force = 0
for joint_id in joint_name2id.values():
    p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL, 
            force=max_force)
const_torque = 0.1

time_elapsed = 0.

while p.isConnected():
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    x, y, z = position
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

    # # position control
    # --------------------------------------------------

    # target_angle = (target_angle + 0.001) % 3.1415
    # print(f"{target_angle=}")

    # p.setJointMotorControl2(robot_id, joint_name2id["w1_joint"],
    #     p.POSITION_CONTROL, target_angle, force=max_force)
    # p.setJointMotorControl2(robot_id, joint_name2id["w2_joint"],
    #     p.POSITION_CONTROL, target_angle, force=max_force)

    # ---------------------------------------------------

    # # velocity control
    # ---------------------------------------------------

    # p.setJointMotorControl2(
    #         bodyUniqueId=robot_id,
    #         jointIndex=joint_name2id["w1_joint"],
    #         controlMode=p.VELOCITY_CONTROL, 
    #         targetVelocity=target_speed, 
    #         force=max_force)
    # p.setJointMotorControl2(
    #         bodyUniqueId=robot_id,
    #         jointIndex=joint_name2id["w2_joint"],
    #         controlMode=p.VELOCITY_CONTROL, 
    #         targetVelocity=target_speed, 
    #         # targetVelocity=target_speed * 0.9, 
    #         force=max_force)

    # =--------------------------------------------------

    # torque control
    # ---------------------------------------------------
    if time_elapsed > 1.5:
        p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_name2id["w1_joint"],
                controlMode=p.TORQUE_CONTROL, 
                force=const_torque)
        p.setJointMotorControl2(
                bodyUniqueId=robot_id,
                jointIndex=joint_name2id["w2_joint"],
                controlMode=p.TORQUE_CONTROL, 
                force=const_torque)

    # ---------------------------------------------------

    p.stepSimulation()
    time_elapsed += dt
    time.sleep(dt)

