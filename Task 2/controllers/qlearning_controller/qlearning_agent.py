#Importing all the required frameworks
import numpy as np
import tensorflow as tf
import keras 
from keras.layers import Input, Dense
from keras.models import Model
from keras.optimizers import Adam
from collections import deque
import matplotlib.pyplot as plt
import random
import gym


# Eager Execution
tf.config.experimental_run_functions_eagerly(True)


#Initializing the Agent class which controls the agents behaviour/improvement in the environment
class Agent(object):


	def __init__(self,state_size,action_size,gamma=0.95,alpha=0.05,epsilon=0.1,memory_size=32,minibatch_size=16,update_freq=10):

		self.state_size=state_size								#Number of features describing each state in the environment
		self.action_size=action_size							#Number of features describing each action in the environment
		self.action_space=np.arange(self.action_size)			#Represents the integer value for each action
		self.gamma=gamma										#Discount factor for future rewards
		self.alpha=alpha										#Learning rate during training
		self.epsilon=epsilon 									#Threshold for exploration while choosing an action
		self.memory_size=memory_size							#Maximum size of the experience replay buffer
		self.minibatch_size=minibatch_size						#Size of minibatch used when updating samples using experience replay
		self.memory=deque(maxlen=self.memory_size)				#Initializing the experience replay buffer as a double ended queue
		self.n_hl1=16											#Number of units in the first hidden layer of the network
		self.n_hl2=16											#Number of units in the second hidden layer of the network
		self.network=self.build_model()							#Building the network that takes states as inputs, and Q-Values as output
		self.target_network=self.build_model()      			#Initializing the target network as the original network
		self.update_freq=update_freq							#The update frequency to equalize the target network to the original one


	#Appending the newly observed data(state,action,reward,next_state,done) to the experience replay
	#If the memeory is full, it automatically deques 
	def append_data(self,data):

		self.memory.append(data)


	#Clearing the experience replay buffer (to be done at the end of each episode)
	def clear_experience_replay(self):

		self.memory.clear()


	#Getting a randomly sampled minibatch from the experience replay
	def get_minibatch(self):

		if len(self.memory) < self.minibatch_size:
			return list(self.memory)
		else:
			return list(self.memory)[-self.minibatch_size:]


	#Initializing the network that outputs Q-Values for each action of a given state
	def build_model(self):

		inputs=Input(shape=[self.state_size],dtype="float32")
		X=Dense(self.n_hl1,kernel_initializer='RandomNormal',activation="relu")(inputs)
		X=Dense(self.n_hl2,kernel_initializer='RandomNormal',activation="relu")(X)
		outputs=Dense(self.action_size,kernel_initializer='RandomNormal')(X)
		model=Model(inputs=inputs,outputs=outputs)
		model.compile(optimizer=Adam(learning_rate=self.alpha),loss="mse")
		return model


	#Following the epsilon greedy policy to choose actions
	def epsilon_greedy_action(self,qvalues):

		A=np.zeros((self.action_size))+self.epsilon/self.action_size
		greedy_action=np.argmax(qvalues[0])
		A[greedy_action]+=1-self.epsilon
		action=np.random.choice(self.action_space,p=A)
		return action


	#Getting the target Q-Values for a particular state, and next_state pair (under a specific action)
	def target_qvalues(self,qvalues,actions,rewards,state_next):

		q_statenext=self.network.predict(state_next.astype("float32"))
		max_q=np.argmax(q_statenext,axis=-1)
		target_qvalues=qvalues.copy()
		for i in range(qvalues.shape[0]):
			target_qvalues[i,actions[i]]=rewards[i]+self.gamma*q_statenext[i,max_q[i]]
		return target_qvalues


	#Updating the network for the minibatch
	def update_network(self,minibatch):

		state_now=[]
		actions=[]
		rewards=[]
		state_next=[]
		for i in range(len(minibatch)):
			state_now.append(minibatch[i][0])
			actions.append(minibatch[i][1])
			rewards.append(minibatch[i][2])
			state_next.append(minibatch[i][3])
		state_now=np.array(state_now)
		actions=np.array(actions)
		rewards=np.array(rewards)
		state_next=np.array(state_next)
		qvalues=self.network.predict(state_now)
		target_qvalues=self.target_qvalues(qvalues,actions,rewards,state_next)
		self.network.fit(state_now.astype("float32"),target_qvalues,epochs=1)


	#Equalizing the target network to the original network at a particular update frequency
	def update_target_network(self):

		self.target_network=keras.models.clone_model(self.network)
		self.target_network.build((None,self.state_size)) 
		self.target_network.compile(optimizer=Adam(learning_rate=self.alpha),loss="mse")
		self.target_network.set_weights(self.network.get_weights())