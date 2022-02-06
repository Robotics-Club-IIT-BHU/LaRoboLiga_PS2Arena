import pybullet as p
import numpy as np
import pybullet_data
import time
import os
import cv2
import helpers as hp

physics_client = p.connect(p.GUI)
p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0,0,-9.8)
parent_dir = os.path.dirname(__file__)+"/.."
p.resetDebugVisualizerCamera(cameraDistance=10,cameraYaw=0,cameraPitch=-60,cameraTargetPosition=[0,0,0])

bot = p.loadURDF(parent_dir+"/rsc/car_with_gripper/urdf/car_with_gripper.urdf",[0,0,0.3],p.getQuaternionFromEuler([0,0,0]))
arena = p.loadURDF(parent_dir+"/rsc/arena/urdf/arena.urdf",useFixedBase = 1)

help=hp.helper(bot)
help.load_balls()

def forward():
    for i in range(0,50):
        help.move(3.14,3.14,3.14,3.14)
        p.stepSimulation()
def right():
    for i in range(0,100):
        help.move(3.14,-3.14,3.14,-3.14)
        p.stepSimulation()
i=0
while True:
  while i>2000 and i<2500:
   help.move(3.14,3.14,3.14,3.14)
   p.stepSimulation()
   i+=1
  help.move(0,0,0,0)
  if i==3000:
    help.open()
  while i>3000 and i<3400:
     help.move(3.14,3.14,3.14,3.14)
     p.stepSimulation()
     i+=1
  help.move(0,0,0,0)
  if i==5000:
   help.close()
  if i==10000:
    help.respawn()
  if i%100==0:
   img = help.get_camera_image(bot)
   cv2.imshow("Camera_image",img)
   cv2.waitKey(1)
  p.stepSimulation()
  i+=1