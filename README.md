# Wormbot

  A snake_type robot with four links and three rvolute joints. Here aim is to make the robot to learn how to move in one direction using deep reinforcemt learning.

# More about the robot:

Hardware
  
  - Its a pair of three servo motors connected in series.

  - There motion is restricted in 2 dimensions ( in vertical plane).

URDF

  to train the agent using pybullet. [URDF_file]()
  
  ![wormbot v2 1](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/4644c392-0eed-4d92-9093-0fb31d0a04f2)
  
  ![image](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/921acde8-b6c0-4d40-84ad-31eda0ac2fd2)

# Training videos
  [training_3_crawling_motion_achived](https://drive.google.com/file/d/1Kd8SF0clKcZI4au7lCsmGXWShbYdoZT3/view?usp=sharing)

  [training_2_short_jump](https://drive.google.com/file/d/1ARtvP1hAl0Pn15aI-MgWzE-2551VbNnb/view?usp=drive_link)
  
  [training_1_long_jump](https://drive.google.com/file/d/1ihMBOIwtcbOCC8-D60fGsqv9S7IOb4bw/view?usp=drive_link)
  
# About the environment

  - state (observation): array of the the servo angles and four orientation Parameters, shape(7,) 

  - Reward: Radial distance moved away form origin gives positive reward
    
  - Action: array of three angles of servo (continuous)

# setup

- For Hardware taining

  - camera is used to track the robot using opencv to calculate the distance moved.

  - Accelerometers, gyroscopes to get orientation is needed (MPU6050)
  
