# Example to spawn the balls and husky at different locations

import gym
import LRL_main_arena
import time
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import os

if __name__=="__main__":
    parent_path = os.path.dirname(os.getcwd())
    os.chdir(parent_path)
    
    """
        The La Robo Liga arena can take three arguments-
            
            ball_locations - A Dictionary data type (see implementation below) - If you want to spawn the balls at different locations. 
            husky_pos - python list or numpy array - If you want to spawn your husky at a differnet position.
            husky_orn - Quaterions (see implementation below) - If you want to change the side husky is facting at the start of round.

        These arguments by default are none, So the arena is loaded with default arguments in the arena class.
        By passing these arguments, they can be customized.
    """
    new_balls_location = dict({
                'red'    : [6,6,1.5],
                'yellow' : [-6,6,1.5],
                'blue'   : [-6,-6,1.5],
                'purple' : [6,-6,1.5]
            })

    new_husky_pos = [0,-8,0.3]
    new_husky_orn = p.getQuaternionFromEuler([0,0,-np.pi/2])  

    env = gym.make(
            "la_robo_liga_arena-v0",
            ball_locations = new_balls_location,
            husky_pos = new_husky_pos,
            husky_orn = new_husky_orn
        )

    for i in range(2000):
        p.stepSimulation()
        time.sleep(1./240.)

    p.disconnect()    