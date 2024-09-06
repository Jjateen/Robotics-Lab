# Importing all the required frameworks
import numpy as np
import tensorflow as tf
import keras
from keras.layers import Input, Dense
from keras.models import Model
import keras.backend as k
from keras.optimizers import Adam
import matplotlib.pyplot as plt
import random
import gym


# Eager Execution
tf.config.experimental_run_functions_eagerly(True)


#Initializing the Agent class which controls the agents behaviour/improvement in the environment
class Agent(object):


	def __init__(self,state_size,action_size,gamma=0.95,alpha=0.05,epsilon=0.1,update_freq=5):

		self.state_size=state_size								#Number of features describing each state in the environment
		self.action_size=action_size							#Number of features describing each action in the environment
		self.gamma=gamma										#Discount factor for future rewards
		self.alpha=alpha										#Learning rate during training
		self.epsilon=epsilon									#Threshold for exploration while choosing an action
		self.n_hl1=3											#Number of units in the first hidden layer of the network
		self.n_hl2=3											#Number of units in the second hidden layer of the network
		self.update_freq=update_freq							#Number of episodes after which the target network takes the value of the first
		self.network1,self.network2=self.build_network()		#Building the network that takes states as inputs, and stochastic probabilities as output
		self.episode_buffer = []								#Keeps track of [current state, action taken, reward recieved] for each episode
		self.count=0


	#A function to initialise/construct the neural network to calcuate stochastic action probabilities
	def build_network(self):

		inputs=Input(shape=[self.state_size])
		reward=Input(shape=[1])
		X=Dense(self.n_hl1, activation='relu')(inputs)
		X=Dense(self.n_hl2, activation='relu')(X)
		outputs=Dense(self.action_size, activation='softmax')(X)
		model1=Model(inputs=[inputs,reward],outputs=outputs)

		def custom_loss(y_pred,y_true):
			probs=k.clip(y_pred,1e-10,1-1e-10)
			return -k.mean(k.log(probs)*y_true)*reward

		model1.compile(optimizer=Adam(learning_rate=self.alpha),loss=custom_loss)

		model2=Model(inputs=inputs,outputs=outputs)

		return model1,model2


	#A function to calculate the discounted return for a particular timestep in an episode
	def discounted_returns(self,rewards):

		discounted_returns=np.zeros((len(rewards)))
		current_return=0
		for t in reversed(range(len(rewards))):
			current_return=rewards[t]+self.gamma*current_return
			discounted_returns[t]=current_return
		return discounted_returns


	#Updating/Fitting the agent's network (at the end of every episode)
	def update_network(self,episode):

		states=[]
		actions=[]
		rewards=[]
		for i in range(len(episode)):
			states.append(episode[i][0])
			actions.append(episode[i][1])
			rewards.append(episode[i][2])
		states=np.array(states)
		actions=np.eye(self.action_size)[np.array(actions)]
		returns=self.discounted_returns(rewards)
		self.network1.fit([states,returns],actions)
		if self.count%self.update_freq==0:
			self.network2=keras.models.clone_model(self.network1)
			self.network2.build((None,self.state_size)) 
			self.network2.compile(optimizer=Adam(learning_rate=self.alpha),loss="mse")
			self.network2.set_weights(self.network1.get_weights())
		self.count += 1


	def work(self,agent_input):

		agent_input=np.array(agent_input)
		policy_output=self.network2.predict(agent_input.reshape([1,self.state_size]))
		if np.random.rand(1)>self.epsilon:
			return np.argmax(policy_output[0])
		else:
			return np.random.randint(self.action_size)