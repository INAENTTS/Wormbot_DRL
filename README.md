# Motion_Planing_with_DRL
Aiming to learn deep reinforcement learning and apply it on a bot to achive a optimal crawling action

! About the robot:
Its a pair of three servo motors connected in series..
There motion is restricted in 2 dimensions ( in vertical plane)
  
! About the environment
*state (observation): list of angles of the three motors
*Reward: positive distance moved
*Action: for each motor rotate ( still:0, clocwise:1, anti clockwise:2). example action=[0,0,0] : ( motor 1 still,motor2 still,motor3 still)

!setup
  camera is used to track a point on robot to calculate the distance moved 
