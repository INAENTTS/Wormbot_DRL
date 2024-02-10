# Wormbot_DRL

  A snake type robot with four links and three revolute joints. Aim is to make the robot to learn how to move in one direction using deep reinforcemt learning.

# Result achieved 

  ![Wormbot_hardware_run](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/8cb754b0-67f5-4d00-a3f1-abffe47d0555)


# URDF

  to train the agent using pybullet. [URDF_file]()
  
  ![wormbot v2 1](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/4644c392-0eed-4d92-9093-0fb31d0a04f2)
  ![image](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/921acde8-b6c0-4d40-84ad-31eda0ac2fd2)

# Training Results on Pybullet simulation
  [training_3_crawling_motion_achived](https://drive.google.com/file/d/1Kd8SF0clKcZI4au7lCsmGXWShbYdoZT3/view?usp=sharing)
  
  ![wormbot_crawl](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/7146c6d8-87cf-4cc7-839e-6d919c147aca)

  [training_2_short_jump](https://drive.google.com/file/d/1ARtvP1hAl0Pn15aI-MgWzE-2551VbNnb/view?usp=drive_link)
  
  ![wormbot_shortjump](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/d1cc4d4c-31f0-4411-bcd9-0d4ea863e58a)

  [training_1_long_jump](https://drive.google.com/file/d/1ihMBOIwtcbOCC8-D60fGsqv9S7IOb4bw/view?usp=drive_link)
  
  ![wormbot_longjump](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/0f635251-a08e-492c-ae95-92b0860022a4)
  
# About the environment

  - state (observation): array of the the servo angles and four orientation Parameters, shape(7,) 

  - Reward: Radial distance moved away form origin gives positive reward
    
  - Action: array of three angles of servo (continuous)

# Setup

- For Hardware taining

  - Camera is used to track the robot using opencv to calculate the distance moved.

  - Accelerometers, gyroscopes to get orientation is needed (MPU6050)
 
  ![wormbot_hardware](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/185eb5e8-1ba0-4b7f-bd67-e1493d4c280d)


