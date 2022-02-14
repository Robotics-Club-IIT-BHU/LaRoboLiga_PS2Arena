import gym
import LRL_main_arena

if __name__ == '__main__':
    env = gym.make("la_robo_liga_arena-v0")

    print("\n Quick doc for pre-made functions to interact woth the arena :")
    
    print("move_husky function--")
    print(env.move_husky.__doc__)

    print("open_husky_gripper function--")
    print(env.open_husky_gripper.__doc__)

    print("close_husky_gripper function--")
    print(env.close_husky_gripper.__doc__)
    
    print("get_camera_image function--")
    print(env.get_camera_image.__doc__)

    print("reset function--")
    print(env.reset_arena.__doc__)