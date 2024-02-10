# Wormbot

  A snake type robot with four links and three revolute joints. Aim is to make the robot to learn how to move in one direction using deep reinforcemt learning.

# URDF

  to train the agent using pybullet. [URDF_file]()
  
  ![wormbot v2 1](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/4644c392-0eed-4d92-9093-0fb31d0a04f2)
  
  ![image](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/921acde8-b6c0-4d40-84ad-31eda0ac2fd2)

# Training videos
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

# setup

- For Hardware taining

  - camera is used to track the robot using opencv to calculate the distance moved.

  - Accelerometers, gyroscopes to get orientation is needed (MPU6050)

  ![Screenshot_2024-01-30-01-42-03-79_92460851df6f172a4592fca41cc2d2e6](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/0437134b-e111-46d4-b9ca-8eda9f8f3d30)

  [training_on_hardware_video](https://drive.google.com/file/d/11Kz8HPsa5_QGHIyYMeIaAgJRQUPnJ-T1/view?usp=sharing)

