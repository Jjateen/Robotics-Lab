![Task2](./Task%202.png)

This code implements a reinforcement learning controller for a Cartpole robot simulation in Webots, utilizing Proximal Policy Optimization (PPO) for training the agent to balance a pole on a moving cart. Here’s a breakdown of each component and the training progression:

### Class and Environment Setup

1. **Imports and CartpoleRobot Class Definition**:
   - The code imports necessary modules, including `RobotSupervisorEnv` from DeepBots, which provides a base environment class for robot simulation.
   - The `CartpoleRobot` class is defined by inheriting `RobotSupervisorEnv`. It initializes observation and action spaces using Gym's `Box` and `Discrete` spaces, respectively.

2. **Observation Space**:
   - The observation space consists of four values: 
     - Cart position on the x-axis (normalized to [-1.0, 1.0])
     - Cart velocity on the x-axis
     - Pole angle relative to vertical
     - Angular velocity of the pole endpoint.
   - These values help the agent determine the current state of the cartpole system.

3. **Action Space**:
   - The action space is `Discrete(2)`, meaning the agent has two actions: move the cart left or right.

4. **Sensors and Motors**:
   - The code initializes a position sensor for pole angle detection and defines four wheel motors. These motors are controlled to maintain the balance of the pole.

5. **Reward Function and Episode Termination**:
   - A constant reward of +1 is given for each timestep the pole remains balanced.
   - The episode ends when:
     - The pole angle deviates by more than 15 degrees from vertical.
     - The cart position goes beyond a set threshold on the x-axis.
     - The accumulated episode score exceeds 195, signaling the task's success.

### Agent Training

6. **Training Loop**:
   - A PPO agent is created with the environment's observation and action space details.
   - The training loop initializes `solved` to `False` and an episode counter (`episode_count`), iterating until either the agent solves the environment or reaches the episode limit (2000).

7. **Episode Loop**:
   - For each episode, the environment is reset, and the agent begins interacting with it.
   - At each timestep:
     - The agent selects an action based on the current observation.
     - The environment updates based on the selected action, returning the new state, reward, and done status.
     - A transition object, containing the current state, action, action probability, reward, and new state, is stored in the agent's memory.
     - If the episode ends, the episode score is appended to the score list, and the agent trains on the collected transitions.

8. **Task Solving Condition**:
   - After each episode, the environment checks if the task has been solved by evaluating the average score of the last 100 episodes.
   - If the average score is above 195, the task is considered solved.

### Output and Results

- The code prints each episode’s score during training. By episode 907, the agent consistently achieves a score of 196, indicating it has learned to balance the pole.
- **Errors**: Several Webots-related errors appear, indicating missing proto declarations (e.g., for `TexturedBackground`, `RectangleArena`). These errors are unrelated to the RL training and pertain to Webots' environment setup.

### Key Points in Execution and Terminal Output

1. **Early Episodes**: Initial episodes score low due to the agent’s random exploration in action selection.
2. **Learning Progression**: Scores gradually increase as the agent refines its policy through PPO.
3. **Achieving Consistent Performance**: By episode 907, the agent stabilizes at a score near 196, signifying it has effectively learned to balance the pole on the cart.
4. **Deployment for Testing**: After solving the task, the agent is deployed in testing mode where it selects the maximum probable action rather than sampling, achieving consistent balancing.

This code effectively demonstrates the implementation of a reinforcement learning model to control a robotic cartpole system in Webots, achieving a stable balancing solution through continuous interaction and learning.
