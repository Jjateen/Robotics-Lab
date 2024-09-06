# This Python progtam aims at implementing a controller based on the Deep QLearning Approach to solve the CartPole Problem

# Importing all the necessary frameworks
from deepbots.supervisor.controllers.robot_supervisor import RobotSupervisor
from qlearning_agent import Agent
from gym.spaces import Box, Discrete
import numpy as np


# A function to normalize observations of the environment into a specific range
def normalizeToRange(value, minVal, maxVal, newMin, newMax, clip=False):
    
    value = float(value)
    minVal = float(minVal)
    maxVal = float(maxVal)
    newMin = float(newMin)
    newMax = float(newMax)
    if clip:
        return np.clip((newMax - newMin) / (maxVal - minVal) * (value - maxVal) + newMax, newMin, newMax)
    else:
        return (newMax - newMin) / (maxVal - minVal) * (value - maxVal) + newMax


# The CartPole Environment constructed using the Deepbots Framework
class CartpoleRobot(RobotSupervisor):


    def __init__(self):

        super().__init__()
        self.observation_space = Box(low=np.array([-0.4, -np.inf, -1.3, -np.inf]),
                                     high=np.array([0.4, np.inf, 1.3, np.inf]),
                                     dtype=np.float64)
        self.action_space = Discrete(2)
        self.robot = self.getSelf() 
        self.positionSensor = self.getDevice("polePosSensor")
        self.positionSensor.enable(self.timestep)

        self.poleEndpoint = self.getFromDef("POLE_ENDPOINT")
        self.wheels = []
        for wheelName in ['wheel1', 'wheel2', 'wheel3', 'wheel4']:
            wheel = self.getDevice(wheelName)
            wheel.setPosition(float('inf'))
            wheel.setVelocity(0.0)
            self.wheels.append(wheel)
        self.stepsPerEpisode = 200
        self.episodeScore = 0
        self.episodeScoreList = []


    def get_observations(self):

        cartPosition = normalizeToRange(self.robot.getPosition()[0], -0.4, 0.4, -1.0, 1.0)
        cartVelocity = normalizeToRange(self.robot.getVelocity()[0], -0.2, 0.2, -1.0, 1.0, clip=True)
        poleAngle = normalizeToRange(self.positionSensor.getValue(), -0.23, 0.23, -1.0, 1.0, clip=True)
        endpointVelocity = normalizeToRange(self.poleEndpoint.getVelocity()[4], -1.5, 1.5, -1.0, 1.0, clip=True)
        return [cartPosition, cartVelocity, poleAngle, endpointVelocity]


    def get_reward(self, action=None):

        return 1


    def is_done(self):

        if self.episodeScore > 195.0:
            return True
        poleAngle = round(self.positionSensor.getValue(), 2)
        if abs(poleAngle) > 0.261799388:
            return True
        cartPosition = round(self.robot.getPosition()[2], 2)
        if abs(cartPosition) > 0.39:
            return True        
        return False


    def solved(self):

        if len(self.episodeScoreList) > 100:
            if np.mean(self.episodeScoreList[-100:]) > 195.0:
                return True
        return False


    def get_default_observation(self):

        return [0.0 for _ in range(self.observation_space.shape[0])]


    def apply_action(self, action):

        action = int(action[0])
        if action == 0:
            motorSpeed = 5.0
        else:
            motorSpeed = -5.0
        for i in range(len(self.wheels)):
            self.wheels[i].setPosition(float('inf'))
            self.wheels[i].setVelocity(motorSpeed)


    def render(self, mode='human'):

        print("render() is not used")


    def get_info(self):

        return None


# Initializing the environment and certain performance metrics
env = CartpoleRobot()
agent = Agent(state_size=env.observation_space.shape[0],action_size=env.action_space.n)
solved = False
episodeCount = 0
episodeLimit = 1000


# Training by running multiple episodes
while not solved and episodeCount < episodeLimit:
    observation = env.reset()
    env.episodeScore = 0
    for step in range(env.stepsPerEpisode):
        selectedAction = agent.epsilon_greedy_action(observation)
        qvalues=agent.network.predict(np.array(observation).reshape(1,agent.state_size).astype("float32"))
        selectedAction=agent.epsilon_greedy_action(qvalues)
        newObservation, reward, done, info = env.step([selectedAction])
        agent.append_data([observation,selectedAction,reward,newObservation])
        minibatch=agent.get_minibatch()
        agent.update_network(minibatch)
        if (step+1)%agent.update_freq==0:
            agent.update_target_network()
        env.episodeScore += reward 
        if done:
            env.episodeScoreList.append(env.episodeScore)
            agent.clear_experience_replay()
            solved = env.solved()
            break
        observation = newObservation
    print("Episode #", episodeCount, "score:", env.episodeScore)
    episodeCount += 1
if not solved:
    print("Task is not solved, deploying agent for testing...")
elif solved:
    print("Task is solved, deploying agent for testing...")
observation = env.reset()
while True:
    selectedAction, actionProb = agent.work(observation)
    observation, _, _, _ = env.step([selectedAction])