import pybullet as p
import numpy as np
import pybullet_data
import time
import os
import cv2
class helper:
  def __init__(self,bot):
     self.bot=bot

  def load_balls(self):
     parent_dir = os.path.dirname(__file__)+"/.."
     ball_yellow = p.loadURDF(parent_dir+"/rsc/Balls/ball_yellow.urdf",[0,6,2.0])
     ball_red = p.loadURDF(parent_dir+"/rsc/Balls/ball_red.urdf",[6,0,2.0])
     ball_blue = p.loadURDF(parent_dir+"/rsc/Balls/ball_blue.urdf",[-6,0,2.0])
     ball_purple = p.loadURDF(parent_dir+"/rsc/Balls/ball_purple.urdf",[0,-6,2.0])
     p.changeDynamics(ball_red,-1,lateralFriction=0.1,rollingFriction=0.1)
     p.changeDynamics(ball_yellow,-1,lateralFriction=0.1,rollingFriction=0.1)
     p.changeDynamics(ball_blue,-1,lateralFriction=0.1,rollingFriction=0.1)
     p.changeDynamics(ball_purple,-1,lateralFriction=0.1,rollingFriction=0.1)
     self.yellow=ball_yellow
     self.red=ball_red
     self.blue=ball_blue
     self.purple=ball_purple

  def move(self,lf,rf,lb,rb):
    vels=[lf,-rf,lb,-rb]
    p.setJointMotorControlArray(
                bodyIndex = self.bot,
                jointIndices = [0,1,2,3],
                controlMode = p.VELOCITY_CONTROL,
                targetVelocities = vels
            )

  def get_camera_image(self,bodyIndex):
    width = 800
    height = 800
    fov = 60
    aspect = width / height
    near = 0.02
    far = 50

    orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bodyIndex)[1])
    pos = p.getBasePositionAndOrientation(bodyIndex)[0]
    camera_eye = [pos[0]+0.4*np.cos(orn[2]),pos[1]+0.4*np.sin(orn[2]),pos[2]+1.15*np.cos(orn[0])]
    target_pos = [pos[0]-2*np.cos(orn[2]),pos[1]-2*np.sin(orn[2]),pos[2]+1.15*np.cos(orn[0])]
    view_matrix = p.computeViewMatrix(camera_eye, target_pos, [0, 0, 1])
    projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

    images = p.getCameraImage(width,
                          height,
                          view_matrix,
                          projection_matrix,
                          shadow=True,
                          renderer=p.ER_BULLET_HARDWARE_OPENGL)
    rgba_opengl = np.reshape(images[2], (height, width, 4))
    bgr = rgba_opengl[0:,0:,0:3]   
    return np.flip(bgr,2) 
  
  def open(self):
   for i in range(100):
     p.setJointMotorControl2(self.bot,5,p.POSITION_CONTROL,targetPosition = np.pi/2)
     p.setJointMotorControl2(self.bot,6,p.POSITION_CONTROL,targetPosition = -np.pi/2)
     p.stepSimulation()
     time.sleep(1./240.)
  
  def close(self):
   for i in range(100):
     p.setJointMotorControl2(self.bot,5,p.POSITION_CONTROL,targetPosition = 0)
     p.setJointMotorControl2(self.bot,6,p.POSITION_CONTROL,targetPosition = 0)             
     p.stepSimulation()
     time.sleep(1./240.)
  
  def respawn(self):
     p.resetBasePositionAndOrientation(self.bot,[0,0,0.3],p.getQuaternionFromEuler([0,0,0]))
     p.resetBasePositionAndOrientation(self.yellow,[0,6,2.0],p.getQuaternionFromEuler([0,0,0]))
     p.resetBasePositionAndOrientation(self.red,[6,0,2.0],p.getQuaternionFromEuler([0,0,0]))
     p.resetBasePositionAndOrientation(self.blue,[-6,0,2.0],p.getQuaternionFromEuler([0,0,0]))
     p.resetBasePositionAndOrientation(self.purple,[0,-6,2.0],p.getQuaternionFromEuler([0,0,0]))
     parent_dir = os.path.dirname(__file__)+"/.."
     p.resetDebugVisualizerCamera(cameraDistance=10,cameraYaw=0,cameraPitch=-60,cameraTargetPosition=[0,0,0])
     for i in range(100):
       p.stepSimulation()