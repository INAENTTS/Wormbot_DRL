import gymnasium as gym
import pybullet as p
from stable_baselines3 import A2C,PPO
import numpy as np
from gymnasium import spaces
from time import sleep
import pybullet_data
physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)
planeId = p.loadURDF("plane.urdf")
cubeStartPos = [0,0,0]
cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("wormbot.urdf",cubeStartPos, cubeStartOrientation,flags=p.URDF_USE_INERTIA_FROM_FILE)
# from pyfirmata import Arduino, SERVO
# board = Arduino('COM4')
# board.digital[10].mode=SERVO
# board.digital[11].mode=SERVO
# board.digital[12].mode=SERVO
# sp1=10
# sp2=11
# sp3=12
# def qact(action):
#     board.digital[sp1].write(action[0])
#     board.digital[sp2].write(action[1])
#     board.digital[sp3].write(action[2])
#     sleep(1.0)
# def sact(action,obs):
#     max=np.max(action)
#     opr1=(action[0]-obs[0])/abs((action[0]-obs[0]))
#     opr2=(action[1]-obs[1])/abs((action[1]-obs[1]))
#     opr3=(action[2]-obs[2])/abs((action[2]-obs[2]))
#     for i in range(0,max):
#         if action[0]==obs[0]+i*opr1:
#             opr1=0
#             obs[0]=action[0]
#         if action[1]==obs[1]+i*opr1:
#             opr2=0
#             obs[1]=action[1]
#         if action[2]==obs[2]+i*opr1:
#             opr3=0
#             obs[2]=action[2]
#         if opr1!=0:
#             board.digital[sp1].write(obs[0]+i*opr1)
#         if opr2!=0:
#             board.digital[sp2].write(obs[1]+i*opr2)
#         if opr3!=0:
#             board.digital[sp3].write(obs[2]+i*opr3)
#         sleep(0.005)
def act(action):
    for i in range (100):
        p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=action, positionGains=[0.1,0.1,0.1])
        p.stepSimulation()
        # time.sleep(1./240.)
        pos,orn=p.getBasePositionAndOrientation(robotId)
        return pos
slow=0
T_timesteps=1000000
class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = spaces.Box(low=np.array([-1,-1,-1]), high=np.array([1,1,1]),shape=(3,))
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=np.array([-1,-1,-1,-1,-1,-1,-1]), high=np.array([1,1,1,1,1,1,1]),shape=(7,))
        self.count=0  
        self.nstp=T_timesteps


    def step(self, action):
        pos,orn=p.getBasePositionAndOrientation(robotId)
        #print(action*85+90)
        act=action*1.396
        for i in range (100):
            p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=act,positionGains=[0.05,0.05,0.05])#, positionGains=[0.1,0.1,0.1])
            p.stepSimulation()
            if slow==1:
                sleep(1./240.)
            # sleep(1./240.)
        npos,orn=p.getBasePositionAndOrientation(robotId)
        #((npos[0]-pos[0]+0.01)/abs(npos[0]-pos[0]+0.01))*
        reward=((npos[0]-pos[0]+0.01)/abs(npos[0]-pos[0]+0.01))*((npos[0]-pos[0])**2+(npos[1]-pos[1])**2)**1/2-10*(npos[2]-pos[2])
        # if reward>0:
        #     reward=reward-10*(npos[2]-pos[2])
        #print(reward)
        self.count+=1
        # if self.count%50==0:
        #     print(reward)
        truncated=False
        terminated=False
        if self.count==200:
            truncated=True
            self.nstp=self.nstp-self.count
            if self.nstp%600==0:
                print(self.nstp)
            self.count=0
        obs=np.append(action,orn)
        return np.float32(obs), reward, terminated, truncated, {}

    def reset(self,seed=None, options=None):
        action=np.array([0,0,0], dtype=np.float32)
        #print(action*85+90)
        act=action*3.14/2
        
        for i in range (100):
            p.setJointMotorControlArray(robotId, range(3), p.POSITION_CONTROL,targetPositions=act,positionGains=[0.1,0.1,0.1])#, positionGains=[0.1,0.1,0.1])
            p.stepSimulation()
            if slow==1:
                sleep(1./240.)
        p.resetBasePositionAndOrientation(robotId,cubeStartPos,cubeStartOrientation)
        p.stepSimulation()
        npos,orn=p.getBasePositionAndOrientation(robotId)
       
        #print('reset')
        obs=np.append(action,orn)
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

model = PPO("MlpPolicy", env).learn(total_timesteps=T_timesteps)

# vec_env = model.get_env()Prospective student: Regarding a 6 months research internship from June - December 2024
# obs = vec_env.reset()


slow=1
print("final run")
a=input()
for i in range (0,3):
    obs=env.reset()[0]
    trunc=False
    while not trunc:
        action, _ = model.predict(obs)
        #print(env.step(action))
        obs, reward, done, trunc,info = env.step(action)
        print(reward)
        #vec_env.render("human")
env.close()