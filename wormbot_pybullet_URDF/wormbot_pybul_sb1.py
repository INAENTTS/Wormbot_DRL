import os
import gymnasium as gym
import pybullet as p
from stable_baselines3 import A2C,PPO
import numpy as np
from gymnasium import spaces
from time import sleep
import pybullet_data
###################################################### 
model_dir=f"models/pybul"
logdir=f"logs"
if not os.path.exists(model_dir):
    os.makedirs(model_dir)
if not os.path.exists(logdir):
    os.makedirs(logdir)
######################################################
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0.03]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])#0,-1,0
robotId = p.loadURDF("wormbot.urdf",cubeStartPos, cubeStartOrientation,flags=p.URDF_USE_INERTIA_FROM_FILE)

# def act(action):# to control the angles of the robot 
#     for i in range (100):
#         p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=action, positionGains=[0.1,0.1,0.1])
#         p.stepSimulation()
#         # time.sleep(1./240.)
#         pos,orn=p.getBasePositionAndOrientation(robotId)
#         return pos
slow=0
T_timesteps=10000
class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        # self.action_space = spaces.Box(low=np.array([-1,-1,-1,-1,-1,-1]), high=np.array([1,1,1,1,1,1]),shape=(6,))
        self.action_space = spaces.Box(low=np.array([-1,-1,-1]), high=np.array([1,1,1]),shape=(3,))
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=np.array([-1,-1,-1,-1,-1,-1,-1]), high=np.array([1,1,1,1,1,1,1]),shape=(7,))
        self.count=0  
        self.nstp=T_timesteps


    def step(self, action):
        pos,orn=p.getBasePositionAndOrientation(robotId)
        #print(action*85+90)
        #print(action,action[0:3],action[3:6])
        ang=action[0:3]*1.4835
        #speed=action[3:6]/20+0.05
        speed=[0.05,0.05,0.05]
        preward=0
        for i in range (60):
            p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=ang,positionGains=speed)#, positionGains=[0.1,0.1,0.1])
            p.stepSimulation()
            if slow==1:
                sleep(1./240.)
            # sleep(1./240.)
        npos,orn=p.getBasePositionAndOrientation(robotId)
        #((npos[0]-pos[0]+0.01)/abs(npos[0]-pos[0]+0.01))*
        # reward=((npos[0]-pos[0]+0.01)/abs(npos[0]-pos[0]+0.01))*((npos[0]-pos[0])**2+(npos[1]-pos[1])**2)**1/2
        reward=(((npos[0])**2+(npos[1])**2)-((pos[0])**2+(pos[1])**2))#*100
        #print(reward)
        # if reward>0.01:
        # #print(npos[2],pos[2])
        #     if (npos[2])>0.1 or (pos[2])>0.1:
        #         reward=0
        #         #print('pp')
        # else:
        #     reward=0
        
        self.count+=1
       
        truncated=False
        terminated=False
        if self.count==200:
            truncated=True
            self.nstp=self.nstp-self.count
            if self.nstp%600==0:
                print(self.nstp)
            self.count=0
        obs=np.append(action[0:3],orn)
        return np.float32(obs), reward, terminated, truncated, {}

    def reset(self,seed=None, options=None):
        # action=np.array([1,-1,1,0.5,0.5,0.5], dtype=np.float32)
        # speed=action[3:6]/20+0.05
        action=np.array([0,0,0], dtype=np.float32)
        ang=action[0:3]*1.4835
        speed=[0.05,0.05,0.05]
        p.resetBasePositionAndOrientation(robotId,cubeStartPos,cubeStartOrientation)
        p.stepSimulation()
        for i in range (60):
            p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=ang,positionGains=speed)#, positionGains=[0.1,0.1,0.1])
            p.stepSimulation()
            if slow==1:
                sleep(1./240.)
        npos,orn=p.getBasePositionAndOrientation(robotId)
        # sleep(1)
        obs=np.append(action[0:3],orn)
        info=0
        return np.float32(obs), {}

    def render(self):
        end=0
        return end

    def close(self):
        end=1
        return end
##################################################################################################################
env = CustomEnv()
from stable_baselines3.common.env_checker import check_env
check_env(env)

model = PPO("MlpPolicy", env,verbose=1,tensorboard_log=logdir)

for i in range(1,100):
    model.learn(total_timesteps=T_timesteps,reset_num_timesteps=False,tb_log_name="pybul")
    model.save(f"{model_dir}/{T_timesteps*i}")

# vec_env = model.get_env()Prospective student: Regarding a 3 months research internship from May - July 2024
# obs = vec_env.reset()


slow=1
print("final run")
a=input()
for i in range (0,50):
    obs=env.reset()[0]
    trunc=False
    while not trunc:
        action, _ = model.predict(obs)
        print(action)
        #print(env.step(action))
        obs, reward, done, trunc,info = env.step(action)
        print(reward)
        #vec_env.render("human")
env.close()
