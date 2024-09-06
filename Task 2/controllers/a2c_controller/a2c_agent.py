# Importing all the reqiured frameworks
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


	def __init__(self,state_size,action_size,gamma=0.95,alpha=0.05,epsilon=0.1):

		self.state_size=state_size								#Number of features describing each state in the environment
		self.action_size=action_size							#Number of features describing each action in the environment
		self.gamma=gamma										#Discount factor for future rewards
		self.alpha=alpha										#Learning rate during training
		self.epsilon=epsilon									#Threshold for exploration while choosing an action
		self.n_hl1=3											#Number of units in the first hidden layer of the network
		self.n_hl2=3											#Number of units in the second hidden layer of the network
		self.actor,self.critic,self.policy=self.build_network()	#Building the networks that output different output layers


	#Defining a function that builds the actor, critic, and policy network to train/improve the model
	def build_network(self):

		inputs=Input(shape=[self.state_size])
		advantage=Input(shape=[1])
		X=Dense(self.n_hl1,activation='relu')(inputs)
		X=Dense(self.n_hl2,activation='relu')(X)
		actor_layer=Dense(self.action_size,activation='softmax')(X)
		critic_layer=Dense(1)(X)

		#Defining a custom loss function (in the keras format) for the policy gradient loss
		def custom_loss(y_pred,y_true):
			probs=k.clip(y_pred,1e-10,1-1e-10)
			return -k.mean(k.log(probs)*y_true)*advantage

		#The policy network takes the state and reward obtained, to calculate probabilities for each action(stochastic policy)
		actor_model=Model(inputs=[inputs,advantage],outputs=actor_layer)
		actor_model.compile(optimizer=Adam(learning_rate=self.alpha),loss=custom_loss)

		#The critic network takes the state, and calculates the value of that state
		critic_model=Model(inputs=inputs,outputs=critic_layer)
		critic_model.compile(optimizer=Adam(learning_rate=self.alpha),loss='mean_squared_error')

		#The policy network is like the actor network, but doesnt take any reward, and just predicts the stochastic probabilities
		#Training is not done on this network, but on the actor network instead
		policy_model=Model(inputs=inputs,outputs=actor_layer)

		return actor_model,critic_model,policy_model


	#Choosing a greedy action, except certain times randomly (randomness can be modified with changing epsilon)
	def choose_action(self,state):

		policy=self.policy.predict(np.array(state).reshape([1,self.state_size]))
		if np.random.rand(1)<self.epsilon:
			return np.argmax(policy[0])
		else:
			return np.random.randint(self.action_size)


	#Updating the actor-critic networks after every timestep in an episode
	def update_network(self,state_now,action,reward,state_next):

		state_now=np.array(state_now).reshape([1,self.state_size])
		state_next=np.array(state_next).reshape([1,self.state_size])
		action=(np.eye(self.action_size)[action]).reshape([1,self.action_size])
		state_now_value=self.critic.predict(state_now)
		state_next_value=self.critic.predict(state_next)
		target_value_now=reward+self.gamma*state_next_value
		advantage=target_value_now-state_now_value
		self.actor.fit([state_now,advantage],action)
		self.critic.fit(state_now,target_value_now)