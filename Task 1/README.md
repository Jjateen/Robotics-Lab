### Task 1: RL from Scratch for Obstacle Avoidance
[Task 1 World File](./worlds/my_world.wbt)  

<p align="center">
  <img src="./Task%201.png" alt="Task 1" width="500">
</p>

In this task, we focus on building a robot model in Webots and training a Reinforcement Learning (RL) agent from scratch. The objective is for the robot to navigate through an environment while avoiding obstacles and reaching a predefined goal.

#### Key Steps:
1. **Robot Model Setup**: The robot is designed using Webots, a 3D robot simulator that enables the modeling of complex robotic systems and environments. The world file `my_world.wbt` includes a simple yet effective setup with obstacles placed randomly within the environment.

2. **Reinforcement Learning**: An RL agent is trained to control the robot’s movements. The agent learns to make decisions such as turning, moving forward, or stopping to avoid obstacles based on feedback from its interactions with the environment.

3. **Goal Achievement**: The robot's goal is to navigate from its starting position to a target location. The RL agent is provided with rewards when it successfully avoids obstacles and progresses toward the goal. Penalties are given for collisions with obstacles or deviating from the path.

4. **Training**: The agent undergoes extensive training using a variety of RL algorithms (such as Q-learning or Proximal Policy Optimization) to gradually improve its ability to navigate the environment. The agent learns the optimal policy for robot control, which can be transferred to real-world robotic applications after sufficient training.

5. **Obstacle Avoidance**: A crucial aspect of the training involves the agent learning to detect and avoid obstacles. Sensors, such as proximity sensors, are used to detect obstacles in the robot’s path and inform the agent of its surrounding environment.

Through this task, we develop both the physical robot model in Webots and the underlying machine learning model to create a fully autonomous robot capable of navigating complex environments efficiently.

--- 
