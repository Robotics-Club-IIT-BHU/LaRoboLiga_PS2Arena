#########################################################################################################
################                        Environment class for La Robo Liga               ################
################                      DO NOT change any piece of code in here            ################
#########################################################################################################

import gym
from gym import error, spaces, utils
from gym.utils import seeding
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import random
from os.path import normpath, basename
import time

class LaRoboLigaPs2Arena(gym.Env):
    metadata = {'render.modes': ['human']}
    def __init__(self,ball_locations = None,husky_pos = None,husky_orn = None):
        """
            Class contructor
            
            -Opens up the Simulation
            -Loads the arena
            Arguments:
                    ball locations - 
                        A dictionary with initial co-ordinates of the Colored balls of color 'red', 'yellow', 'blue' and 'purple'.
                        The co-ordinates must be in form of a python list or a numpy array.
                    
                    husky_pos-
                        Position of husky in 3D space

                    husky_orn-
                        Orientation of the husky in 3D spce expressed expressed in Quatrnions         
        """
        p.connect(p.GUI)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0,0,-10)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS,0)
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME,0)

        if ball_locations is not None:
            self.color_balls_location = ball_locations
        else:    
            self.color_balls_location = dict({
                'red'    : [6,0,1.5],
                'yellow' : [0,6,1.5],
                'blue'   : [-6,0,1.5],
                'purple' : [0,-6,1.5]
            })

        if husky_pos is None:
                    self.husky_pos = [0,0,0.3]
        else:
                    self.husky_pos = husky_pos

        if husky_orn is None:
                        self.husky_orn = p.getQuaternionFromEuler([0,0,np.pi])         
        else:
                        self.husky_orn = husky_orn

        self.husky = None
        self.balls = list()

        self.__load_arena()
        self.load_balls()
        self.spawn_husky()
        
        self._width = 600
        self._height = 600

        for i in range(100):            # running the Simulatin for short period to let Everything settle
            p.stepSimulation()
            time.sleep(1./240.)

    def move_husky(self,leftfront,rightfront,leftback,rightback):
        """
            Function to move the husky
            --------------------------
                
            Arguments:

                leftFrontWheel - Velocity of the front left wheel  
                rightFrontWheel - Velocity of the front right wheel  
                leftRearWheel - Velocity of the rear left wheel  
                rightRearWheel - Velocity of the rear right wheel  


            Return Values:

                None

        """
        max_vel = 2
        vels = [leftfront,-rightfront,leftback,-rightback]
        for vel in vels:
            if vel > max_vel : vel = max_vel
            elif vel < -max_vel : vel = -max_vel
            else : pass         
        vels = [leftfront,-rightfront,leftback,-rightback]

        p.setJointMotorControlArray(
                    bodyIndex = self.husky,
                    jointIndices = [0,1,2,3],
                    controlMode = p.VELOCITY_CONTROL,
                    targetVelocities = vels
             )
    def open_husky_gripper(self):
        """
            Function to open the grippers attached in front of husky

            Arguments : 
                    None

            Returns :
                    None
        """
        for i in range(100):
            p.setJointMotorControl2(self.husky,5,p.POSITION_CONTROL,targetPosition = np.pi/2)
            p.setJointMotorControl2(self.husky,6,p.POSITION_CONTROL,targetPosition = -np.pi/2)
            p.stepSimulation()
            time.sleep(1./240.)
    
    def close_husky_gripper(self):
        """
            Function to close the grippers attached in front of husky

            Arguments : 
                    None
                    
            Returns :
                    None
        """        
        for i in range(100):
            p.setJointMotorControl2(self.husky,5,p.POSITION_CONTROL,targetPosition = 0)
            p.setJointMotorControl2(self.husky,6,p.POSITION_CONTROL,targetPosition = 0)             
            p.stepSimulation()
            time.sleep(1./240.)

    def get_camera_image(self):
        """
            Function to get camera feed from the onboard camera on husky.

            Arguments:
                    None
            
            Return Values:
                    numpy array of BGR values
		"""
        fov = 60
        aspect = self._width / self._height
        near = 0.02
        far = 50

        orn = p.getEulerFromQuaternion(p.getBasePositionAndOrientation(self.husky)[1])
        pos = p.getBasePositionAndOrientation(self.husky)[0]
        camera_eye = [pos[0]+0.4*np.cos(orn[2]),pos[1]+0.4*np.sin(orn[2]),pos[2]+1.15*np.cos(orn[0])]
        target_pos = [pos[0]-2*np.cos(orn[2]),pos[1]-2*np.sin(orn[2]),pos[2]+1.15*np.cos(orn[0])]
        view_matrix = p.computeViewMatrix(camera_eye, target_pos, [0, 0, 1])
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)

        images = p.getCameraImage(
                            self._width,
                            self._height,
                            view_matrix,
                            projection_matrix,
                            shadow=True,
                            renderer=p.ER_BULLET_HARDWARE_OPENGL
                        )
        rgba_opengl = np.reshape(images[2], (self._height, self._width, 4))
        bgr = np.uint8(bgr)
        bgr = cv2.cvtColor(rgba_opengl[:,:,0:3],cv2.COLOR_BGR2RGB)   
        return bgr

    def load_balls(self):
        """
            Function to load the Colored balls in the arena

            Arguments:
                custom_ball_locations:
                        A dictionary with initial co-ordinates of the Colored balls of color 'red', 'yellow', 'blue' and 'purple'.
                        The co-ordinates must be in form of a python list or a numpy array.
            
            returns:
                    None
        """

        for color in self.color_balls_location:
            id = p.loadURDF('rsc/Balls/ball_'+color+'.urdf',self.color_balls_location[color])
            p.changeDynamics(id,-1,lateralFriction=0.1,rollingFriction=0.1)
            self.balls.append(id)

    def spawn_husky(self):
        self.husky = p.loadURDF('rsc/car_with_gripper/urdf/car_with_gripper.urdf',self.husky_pos,self.husky_orn) 

    def __load_arena(self):
        """
            Function to load the arena 

            Arguments:
                    None
            returns:
                    None                        
            
        """
        p.loadURDF('rsc/arena/urdf/arena.urdf',useFixedBase = 1)      


    def reset_arena(self):
        """
            Function to reset the postions of the husky and the balls in arena

            Arguments:
                     None

            returns:
                    None         
        """
        p.removeBody(self.husky)
        for id in self.balls:
            p.removeBody(id)
            
        self.balls.clear()
        self.spawn_husky()
        self.load_balls()  
