# Wormbot

A snake_type robot with four links and three rvolute joints. Here aim is to make the robot to learn how to move in one direction using deep reinforcemt learning.

# More about the robot:

Hardware
  
  Its a pair of three servo motors connected in series.

  There motion is restricted in 2 dimensions ( in vertical plane).

URDF

  to train the agent using pybullet. [URDF_file]()
  
  ![Alt text](![Screenshot (17)](https://github.com/INAENTTS/Wormbot_DRL/assets/120380768/7868a1d3-4d5c-4331-95b3-7ee004e510e7))
  
# About the environment

  *state (observation): array of the the servo angles and four orientation Parameters, shape(7,) 

  *Reward: distance moved

  *Action: array of three angles of servo (continuous)

# setup

for Hardware taining

  camera is used to track the robot using opencv to calculate the distance moved.
  
