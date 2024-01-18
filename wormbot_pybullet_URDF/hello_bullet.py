import pybullet as p
import time
import pybullet_data
import numpy as np
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("wormbot.urdf",cubeStartPos, cubeStartOrientation, flags=p.URDF_USE_INERTIA_FROM_FILE)
                   # useMaximalCoordinates=1, ## New feature in Pybullet
pos,orn=p.getBasePositionAndOrientation(robotId)
while True:
    for i in range (100):
        p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=[0,0,3.14/2],positionGains=[0.05,0.05,0.05])
        p.stepSimulation()
        time.sleep(1./240.)
        pos,orn=p.getBasePositionAndOrientation(robotId)
        # a = p.getQuaternionFromEuler(orn)
        # kk=np.array([orn,[1,1,1]])
        kk=np.append(orn,[1,1,1])
        print(kk)
    for i in range (100):
        p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=[0,0,-3.14/2],positionGains=[0.05,0.05,0.05])
        p.stepSimulation()
        time.sleep(1./240.)
        pos,orn=p.getBasePositionAndOrientation(robotId)
        # print(pos)
cubePos, cubeOrn = p.getBasePositionAndOrientation(robotId)
print(cubePos,cubeOrn)
p.disconnect()

