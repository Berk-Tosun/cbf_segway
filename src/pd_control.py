import pybullet as p
import pybullet_data
import matplotlib.pyplot as plt
import numpy as np

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

# enable torque control
max_force = 0
for joint_id in joint_name2id.values():
    p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_id,
            controlMode=p.VELOCITY_CONTROL, 
            force=max_force)
torque_target = 0
torque_limit = 0.3

# aim to stabilize pitch
target_pitch = 0
k_p = 0.2
k_d = 0.1
# k_d = 1e-6

trqs_target = []
trqs_applied = []
trqs_kp = []
trqs_kd = []

ptchs = []
ptchs_dot = []

prev_pitch = None
time_elapsed = 0
# while p.isConnected():
while time_elapsed < 8:
    position, orientation = p.getBasePositionAndOrientation(robot_id)
    x, y, z = position
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)

    # if prev_pitch == None:
    #     prev_pitch = pitch
    #     pitch_dot = 0
    # else:
    #     pitch_dot = (pitch - prev_pitch) / dt
    #     prev_pitch = pitch
    _, ang_vel_base = p.getBaseVelocity(robot_id)
    pitch_dot = ang_vel_base[1]

    torque_target_kp = k_p * (pitch - target_pitch)
    torque_target_kd = k_d * pitch_dot
    torque_target = torque_target_kp + torque_target_kd 
    torque_applied = np.clip(torque_target, -torque_limit, torque_limit)

    trqs_target.append(torque_target)
    trqs_applied.append(torque_applied)
    
    trqs_kp.append(torque_target_kp)
    trqs_kd.append(torque_target_kd)

    ptchs.append(pitch)
    ptchs_dot.append(pitch_dot)

    p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_name2id["w1_joint"],
            controlMode=p.TORQUE_CONTROL, 
            force=torque_applied)
    p.setJointMotorControl2(
            bodyUniqueId=robot_id,
            jointIndex=joint_name2id["w2_joint"],
            controlMode=p.TORQUE_CONTROL, 
            force=torque_applied)

    p.stepSimulation()
    time_elapsed += dt
    time.sleep(dt)
    # if time_elapsed % 0.2 < 0.01:
    #     print(roll, pitch, yaw)     

_, axs = plt.subplots(2, 2)
t = np.linspace(0, time_elapsed, len(trqs_applied)) 

axs[0, 0].plot(t, np.array(trqs_applied), label='applied')
axs[0, 0].plot(t, np.array(trqs_target), "--", label='target')
axs[0, 0].set_title("Joint Torque")
axs[0, 0].set_xlabel("time")
axs[0, 0].legend()

axs[0, 1].plot(t, trqs_kp, label="k_p")
axs[0, 1].plot(t, trqs_kd, "--", label="k_d")
axs[0, 1].set_title("Controller Contributions")
axs[0, 1].set_xlabel("time")
axs[0, 1].legend()

axs[1, 0].plot(t, ptchs, label="pitch")
axs[1, 0].plot(t, ptchs_dot, "--", label="pitch dot")
axs[1, 0].set_title("States")
axs[1, 0].set_xlabel("time")
axs[1, 0].legend()

axs[1, 1].scatter(ptchs, ptchs_dot, c=t)
axs[1, 1].set_title("States (Phase plane)")
axs[1, 1].set_xlabel("Pitch")
axs[1, 1].set_ylabel("Pitch dot")
axs[1, 1].grid()

plt.tight_layout()
plt.show()

