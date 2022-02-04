import pybullet as p
import numpy as np
import pybullet_data
import time
import os
import cv2

def get_camera_image(bodyIndex):
    width = 512
    height = 512

    fov = 60
    aspect = width / height
    near = 0.02
    far = 30

    orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bodyIndex)[1])
    pos = p.getBasePositionAndOrientation(bodyIndex)[0]
    camera_eye = [pos[0]+0.4*np.cos(orn[2]),pos[1]+0.4*np.sin(orn[2]),pos[2]+1.0*np.cos(orn[0])]
    target_pos = [pos[0]-2*np.cos(orn[2]),pos[1]-2*np.sin(orn[2]),pos[2]+1.0*np.cos(orn[0])]
    print(orn[2])
    view_matrix = p.computeViewMatrix(camera_eye, target_pos, [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    # Get depth values using the OpenGL renderer
    print(view_matrix)
    print(projection_matrix)
    images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba_opengl = np.reshape(images[2], (height, width, 4)) * 1. / 255.
    bgr = rgba_opengl[0:,0:,0:3]   
    return np.flip(bgr,2) 

physics_client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)

parent_dir = os.path.dirname(__file__)+"/.."

#plane = p.loadURDF("plane.urdf")
print(parent_dir)

orn = p.getQuaternionFromEuler([0,0,np.pi])
bot = p.loadURDF(parent_dir+"/rsc/car_with_gripper/urdf/car_with_gripper.urdf",[0,0,0.5],orn)
arena = p.loadURDF(parent_dir+"/rsc/arena/urdf/arena.urdf",useFixedBase = 1,globalScaling = 1.2)

ball_green = p.loadURDF(parent_dir+"/rsc/Balls/ball_green.urdf",[0,6,2.0])
ball_red = p.loadURDF(parent_dir+"/rsc/Balls/ball_red.urdf",[6,0,2.0])
ball_blue = p.loadURDF(parent_dir+"/rsc/Balls/ball_blue.urdf",[-6,0,2.0])
ball_purple = p.loadURDF(parent_dir+"/rsc/Balls/ball_purple.urdf",[0,-6,2.0])

num_joints = p.getNumJoints(bot)

for i in range(num_joints):
    print(p.getJointInfo(bot,i))
vels = [0,0,0,0]
#p.setRealTimeSimulation(1)

i = 0
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
            p.stepSimulation()
            time.sleep(1./240.)    
        if(k == p.B3G_DOWN_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [-3.14,3.14,-3.14,3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )              
            p.stepSimulation()
            time.sleep(1./240.)    
        if(k == p.B3G_LEFT_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [-3.14,-3.14,-3.14,-3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )              
            p.stepSimulation()
            time.sleep(1./240.)    
        if(k == p.B3G_RIGHT_ARROW and (v & p.KEY_IS_DOWN)):
            vels = [3.14,3.14,3.14,3.14]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )              
            p.stepSimulation()
            time.sleep(1./240.)    
        if(k == ord('o') and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(bot,5,p.POSITION_CONTROL,targetPosition = np.pi/2)
            p.setJointMotorControl2(bot,6,p.POSITION_CONTROL,targetPosition = -np.pi/2)
            p.stepSimulation()
            time.sleep(1./240.)
        if(k == ord('c') and (v & p.KEY_IS_DOWN)):
            p.setJointMotorControl2(bot,5,p.POSITION_CONTROL,targetPosition = 0)
            p.setJointMotorControl2(bot,6,p.POSITION_CONTROL,targetPosition = 0)             
            p.stepSimulation()
            time.sleep(1./240.)
        if(v & p.KEY_WAS_RELEASED):
            vels = [0,0,0,0]
            p.setJointMotorControlArray(
                bodyIndex = bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )
    p.stepSimulation()
    if(i%33 == 0):
        img = get_camera_image(bot)
        cv2.imshow("Camera_image",img)
        cv2.waitKey(1)
    i += 1    