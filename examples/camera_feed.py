# Example use of get_camera_image() function

import gym
import LRL_main_arena
import time
import pybullet as p
import cv2
import os

if __name__ == '__main__':
    parent_path = os.path.dirname(os.getcwd())
    os.chdir(parent_path)
    env = gym.make('la_robo_liga_arena-v0')
    time.sleep(1)
    for i in range(2000):
        if(i%10 == 0):                          # rendering every 10 iterations instead of each
            img = env.get_camera_image()        # to decrease computational load
        p.stepSimulation()
        #time.sleep(1./240.)
        cv2.imshow('On board Camera',img)
        cv2.waitKey(1)
    time.sleep(2)
    p.disconnect()
    cv2.destroyAllWindows()