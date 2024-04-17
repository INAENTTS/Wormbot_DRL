import gymnasium as gym
from stable_baselines3 import A2C,PPO
import numpy as np
from gymnasium import spaces
from pyfirmata import Arduino, SERVO
from time import sleep
#############################
import os
os.environ["BLINKA_MCP2221"]="Adafruit_blinka"
import board
import digitalio
from analogio import AnalogIn
import adafruit_mpu6050
i2c = board.I2C()
mpu = adafruit_mpu6050.MPU6050(i2c)
def orientation():
    acc=mpu.acceleration[2]
    if acc>0:
        s=1
    else:
        s=-1
    return s
############################################################
arduino = Arduino('COM4')
arduino.digital[10].mode=SERVO
arduino.digital[11].mode=SERVO
arduino.digital[12].mode=SERVO
arduino.digital[2].write(1)
sp1=10
sp2=11
sp3=12
def wakeup():
    arduino.digital[2].write(0)
    sleep(0.05)
    arduino.digital[2].write(1)
def qact(action):
    action=80*action+90
    arduino.digital[sp1].write(action[0])
    arduino.digital[sp2].write(action[1])
    arduino.digital[sp3].write(action[2])
    sleep(1.0)
def sact(action,obs):
    action=80*action+90
    # print(action)
    ta1=action[0]
    ta2=action[1]
    ta3=action[2]
    a1=ta1-obs[0]
    a2=ta2-obs[1]
    a3=ta3-obs[2]
    # print(obs)
    # board.digital[sp1].write(opr1)
    # board.digital[sp2].write(opr2)
    # board.digital[sp3].write(opr3)
    # sleep(0.005)
    # print(arduino.digital[sp1].read(),arduino.digital[sp2].read(),arduino.digital[sp3].read())
    # maxa=np.max([a1,a2,a3])
    for i in range(5):
        if a1!=0:
            a1=a1-(a1/5)*a1/abs(a1+0.001)
            # print(a1)
            arduino.digital[sp1].write(ta1-a1)
        if a2!=0:
            a2=a2-(a2/5)*a2/abs(a2)
            arduino.digital[sp2].write(ta2-a2)
        if a3!=0:
            a3=a3-(a3/5)*a3/abs(a3)
            arduino.digital[sp3].write(ta3-a3)
        sleep(0.05)
       

########################################################
import cv2
tracker_types = ['BOOSTING', 'MIL','KCF', 'TLD', 'MEDIANFLOW', 'GOTURN', 'MOSSE', 'CSRT']
tracker_type = tracker_types[0]

if tracker_type == 'BOOSTING':
    tracker = cv2.legacy.TrackerBoosting_create()
if tracker_type == 'MIL':
    tracker = cv2.TrackerMIL_create() 
if tracker_type == 'KCF':
    tracker = cv2.TrackerKCF_create() 
if tracker_type == 'TLD':
    tracker = cv2.legacy.TrackerTLD_create() 
if tracker_type == 'MEDIANFLOW':
    tracker = cv2.legacy.TrackerMedianFlow_create() 
if tracker_type == 'GOTURN':
    tracker = cv2.TrackerGOTURN_create()
if tracker_type == 'MOSSE':
    tracker = cv2.legacy.TrackerMOSSE_create()
if tracker_type == "CSRT":
    tracker = cv2.TrackerCSRT_create()

# Get the video file and read it
video = cv2.VideoCapture(0)
#video = cv2.VideoCapture("walking.mp4")
ret, frame = video.read()

frame_height, frame_width = frame.shape[:2]
# Resize the video for a more convinient view
frame = cv2.resize(frame, [frame_width//2, frame_height//2])
# Initialize video writer to save the results
output = cv2.VideoWriter(f'{tracker_type}.avi', 
                         cv2.VideoWriter_fourcc(*'XVID'), 60.0, 
                         (frame_width//2, frame_height//2), True)
if not ret:
    print('cannot read the video')

# Select the bounding box in the first frame
bbox = cv2.selectROI(frame, True)
ret = tracker.init(frame, bbox)
########################################################
def getx():
    x1=np.zeros(2,)
    while True:
        ret, frame = video.read()
        frame = cv2.resize(frame, [frame_width//2, frame_height//2])
        if not ret:
            print('something went wrong')
            break
        timer = cv2.getTickCount()
        ret, bbox = tracker.update(frame)
        x1[0]=bbox[0]
        x1[1]=bbox[1]
        fps = cv2.getTickFrequency() / (cv2.getTickCount() - timer)
        if ret:
            p1 = (int(bbox[0]), int(bbox[1]))
            p2 = (int(bbox[0] + bbox[2]), int(bbox[1] + bbox[3]))
            cv2.rectangle(frame, p1, p2, (255,0,0), 2, 1)
        else:
            cv2.putText(frame, "Tracking failure detected", (100,80), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75,(0,0,255),2)
        cv2.putText(frame, tracker_type + " Tracker", (100,20), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        cv2.putText(frame, "FPS : " + str(int(fps)), (100,50), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.75, (50,170,50),2)
        cv2.imshow("Tracking", frame)
        output.write(frame)
        k = cv2.waitKey(1) & 0xff
        if k == 27 : break
        if cv2.waitKey(1) & 0xFF == ord('q') or (x1[0]==250 or x1[0]==0 or x1[1]==250 or x1[1]==0):
            while cv2.waitKey(1) & 0xFF != ord('w'):
                print("pp")
        break
    return x1
########################################################
obs=np.zeros(3)
########################################################


class CustomEnv(gym.Env):
    """Custom Environment that follows gym interface."""

    metadata = {"render_modes": ["human"], "render_fps": 30}

    def __init__(self):
        super().__init__()
        # Define action and observation space
        # They must be gym.spaces objects
        # Example when using discrete actions:
        self.action_space = spaces.Box(low=np.array([-1,-1,-1]), high=np.array([1,1,1]), shape=(3,))
        # Example for using image as input (channel-first; channel-last also works):
        self.observation_space = spaces.Box(low=np.array([-1,-1,-1,-1]), high=np.array([1,1,1,1]),shape=(4,), dtype=np.float32)
        self.count=0
        self.obs=np.zeros(4)
    def step(self, action):
        #print(action*85+90)
        xy=getx()
        a_act=action
        sact(a_act,self.obs)
        nxy=getx()
        ###########################
        delta=nxy-xy
        dis=(delta[0]**2+delta[1]**2)*(delta[0])/(abs(delta[0])+0.001)
        reward=dis
        ###########################
        print(reward)
        self.count+=1
        truncated=False
        terminated=False
        if self.count==200:
            truncated=True
            self.count=0
        ori=np.zeros(1)
        ori[0]=orientation()
        self.obs=np.concatenate((action,ori), dtype=np.float32)
        return self.obs, reward, terminated, truncated, {}

    def reset(self, seed=None, options=None):
        action=np.array([0,0,0], dtype=np.float32)
        #print(action*85+90)
        a_act=action
        qact(a_act)
        print('reset')
        info=0
        ori=np.zeros(1)
        ori[0]=orientation()
        self.obs=np.concatenate((action,ori), dtype=np.float32)
        return self.obs, {}

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

model = A2C("MlpPolicy", env).learn(total_timesteps=100000)

# vec_env = model.get_env()
# obs = vec_env.reset()

obs=env.reset()[0]
trunc=False
while not trunc:
    action, _ = model.predict(obs)
    #print(env.step(action))
    obs, reward, done, trunc,info = env.step(action)
    #vec_env.render("human")
env.close()
