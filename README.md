<p align="center">
 <img  width="400" height="400" src="https://github.com/Robotics-Club-IIT-BHU/Vision2_20_Areana/blob/main/media/robo.jpg"><br>
  <i>presents:</i>  
</p>

# La-Robo-Liga (PS2)

### Welcome to the second problem statement of La-Robo-Liga 2022.   
We are glad that you made it here and hope to see you cruise through the task.  
This repository holds the **official arena for the event** which will be used for evaluation of the submissions by the participants.

## Installation Guidelines  
Clone the repository or download the zip file and extract it.

## Getting started
There are a few helper functions in [env_class.py](https://github.com/Robotics-Club-IIT-BHU/FreshersEvent_PS2Arena/blob/HD/test_scripts/env_class.py) which we have made to assist you in giving velocity commands to the bot and opening and closing the gripper
   * `env.get_camera_image()`  
      This will return an RGB image from the camera placed in front of the bot husky just above the gripper origin.  
      
   * `env.move_husky(frontleftwheelvelocity,frontrightwheelvelocity,backleftwheelvelocity,backrightwheelvelocity)`  
      This is used to control the velocity of each wheel of husky individually.
      
   * `env.open_husky_gripper()`  
      This will be used to open the gripper arms of husky.
      
   * `env.close_husky_gripper()`  
      This will be used to close the gripper arms of husky.
      
   * `env.reset()`
      This will reset the whole arena. **This function cannot be used for your final submission.** 
      
## A sample image from the Camera Feed
<p align="center">
 <img  width="400" height="400" src="https://github.com/Robotics-Club-IIT-BHU/FreshersEvent_PS2Arena/blob/HD/Sample_Camera_Image.png"><br>
</p>
