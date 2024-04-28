# Wormbot_DRL

  A snake type robot with four links and three revolute joints. Aim is to make the robot to learn how to move in one direction using deep reinforcement learning.

| Result achieved using RL| ![Wormbot_hardware_run](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/8cb754b0-67f5-4d00-a3f1-abffe47d0555) |
|------------------|-----------|

# Mean Reward

  <img width="785" alt="image" src="https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/41fd4942-ea27-4e03-8c11-c9a24d69df39">


# URDF

  to train the agent using pybullet. [URDF_file](https://github.com/INAENTTS/Wormbot_DRL/tree/main/wormbot_URDF)
  
  ![wormbot v2 1](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/4644c392-0eed-4d92-9093-0fb31d0a04f2)
  ![image](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/921acde8-b6c0-4d40-84ad-31eda0ac2fd2)

# Training Results on Pybullet simulation

  |  Training | Output |
  | --------------------------------- | ------------- |
  | First Training output  | ![wormbot_longjump](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/0f635251-a08e-492c-ae95-92b0860022a4)   |
  | Second, with constrain on reward  | ![wormbot_shortjump](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/d1cc4d4c-31f0-4411-bcd9-0d4ea863e58a)  |
  | final, reduced joint angular speed| ![wormbot_crawl](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/7146c6d8-87cf-4cc7-839e-6d919c147aca)      |
  
# About the environment

  | State Space  | Three angles of joints and four orientation Parameters (continuous)| shape(7,) |
  | ------------- | ------------- |------------|
  | Action  | angles of servo (-85 <sup>o</sup>, +85 <sup>o</sup>) normalised to (-1, 1) (continuous) | shape(3,)|
  | Reward  | Distance moved form the origin ( away : +ve , toward origin : -ve)  | shape(1,)|

# Setup

- For Hardware taining

  - Camera is used to track the robot using opencv to calculate the distance moved.

  - Accelerometers, gyroscopes to get orientation is needed (MPU6050)
 
  ![wormbot_hardware](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/185eb5e8-1ba0-4b7f-bd67-e1493d4c280d)


