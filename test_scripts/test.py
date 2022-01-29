import pybullet as p
import numpy as np
import pybullet_data
import time
import os

physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

parent_dir = os.path.dirname(__file__)+"/.."

plane = p.loadURDF("plane.urdf")
print(parent_dir)
bot = p.loadURDF(parent_dir+"/rsc/car_with_gripper/urdf/car_with_gripper.urdf",[0,0,0.5])

ball_green = p.loadURDF(parent_dir+"/rsc/Balls/ball_green.urdf",[-10,0,2.0])
ball_red = p.loadURDF(parent_dir+"/rsc/Balls/ball_red.urdf",[-5,5,2.0])
ball_blue = p.loadURDF(parent_dir+"/rsc/Balls/ball_blue.urdf",[-5,-5,2.0])

num_joints = p.getNumJoints(bot)

for i in range(num_joints):
    print(p.getJointInfo(bot,i))
vels = [0,0,0,0]
p.setRealTimeSimulation(1)
while True:
    keys = p.getKeyboardEvents()
    for k,v in keys.items():
        if(k == p.B3G_UP_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [3.14,-3.14,3.14,-3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )    
            #p.stepSimulation()    
        if(k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [-3.14,3.14,-3.14,3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )              
            #p.stepSimulation()    
        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [-3.14,-3.14,-3.14,-3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )              
            #p.stepSimulation()    
        if(k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [3.14,3.14,3.14,3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )              
            #p.stepSimulation()    
        if(k == ord('o') and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(bot,5,p.POSITION_CONTROL,targetPosition = 1.57)
            p.setJointMotorControl2(bot,6,p.POSITION_CONTROL,targetPosition = -1.57)
            #p.stepSimulation()
        if(k == ord('c') and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(bot,5,p.POSITION_CONTROL,targetPosition = 0)
            p.setJointMotorControl2(bot,6,p.POSITION_CONTROL,targetPosition = 0)             
            #p.stepSimulation()
        if(v & p.KEY_WAS_RELEASED):
            vels = [0,0,0,0]
    p.setJointMotorControlArray(
        bodyIndex = bot,
        jointIndices = [0,1,2,3],
        controlMode = p.VELOCITY_CONTROL,
        targetVelocities = vels
    )

    p.stepSimulation()
    time.sleep(1./240.)