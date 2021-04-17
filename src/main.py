import pybullet as p
import pybullet_data

import time
import pathlib

dt = 1/25

p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.resetSimulation()

p.loadURDF('plane.urdf')

cubeStartPos = [0, 0, 1.13]
cubeStartOrientation = p.getQuaternionFromEuler([0., 0, 0])
urdf_file = pathlib.Path("./URDF/cyberpod.urdf")
botId = p.loadURDF(urdf_file.absolute().as_posix(), cubeStartPos, 
    cubeStartOrientation)

p.setGravity(0, 0, -9.807)
p.setTimeStep(dt)

# Run the simulation for a fixed amount of steps.
while p.isConnected():
    position, orientation = p.getBasePositionAndOrientation(botId)
    x, y, z = position
    roll, pitch, yaw = p.getEulerFromQuaternion(orientation)
    p.stepSimulation()
    time.sleep(dt)

