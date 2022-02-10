# Example to use the open_husky_gripper() and close_husky_gripper() functions

import gym
import LRL_main_arena
import time
import pybullet as p
import pybullet_data
import cv2
import os

if __name__=="__main__":
    parent_path = os.path.dirname(os.getcwd())
    os.chdir(parent_path)

    env = gym.make("la_robo_liga_arena-v0")
    time.sleep(2)
    env.open_husky_gripper()
    time.sleep(2)
    env.close_husky_gripper()
    time.sleep(2)

    p.disconnect()