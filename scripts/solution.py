# This is just a basic template for solution

import LRL_main_arena
import gym
import time
import pybullet as p
import cv2
import os
import numpy as np

if __name__ == '__main__':
    parent_path = os.path.dirname(os.getcwd()) # This line is to switch the directories for getting resources.
    os.chdir(parent_path)                      # you don't need to change anything in here.    

    env = gym.make("la_robo_liga_arena-v0")    # This loads the arena.
   
    # Initialize your global variables/constants here. 
    
    
    while True:                                # main loop to run the simulation.             
        p.stepSimulation()                     # capture images, extract data from them and take actions accordingly to complete the obj.
