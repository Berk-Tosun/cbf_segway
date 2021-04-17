import pybullet as p
import pybullet_data

import time
import pathlib

dt = 1e-3

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

target_angle = 0

while p.isConnected():
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    x, y, z = position
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

    target_angle = (target_angle + 0.001) % 3.1415
    print(f"{target_angle=}")

    p.setJointMotorControl2(robot_id, joint_name2id["w1_joint"],
        p.POSITION_CONTROL, target_angle)
    p.setJointMotorControl2(robot_id, joint_name2id["w2_joint"],
        p.POSITION_CONTROL, target_angle)

    p.stepSimulation()
    time.sleep(dt)

