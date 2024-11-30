### Task 2: CartPole Ballance using Deepbots

[Task 2 World File](./worlds/CartPoleWorld.wbt)

<p align="center">
    <img src="./Task%202.png" alt="Task 2" width="500">
</p>

### 1. Imports and Environment Setup
```python
from deepbots.supervisor.controllers.robot_supervisor_env import RobotSupervisorEnv
from utilities import normalize_to_range
from PPO_agent import PPOAgent, Transition
from gym.spaces import Box, Discrete
import numpy as np
```

The script uses the `deepbots` library, which provides utilities for integrating reinforcement learning with Webots. It uses `normalize_to_range` from the `utilities` module to scale values, and imports `PPOAgent` and `Transition` from a custom PPO implementation. `gym.spaces` is used to define observation and action spaces.

### 2. `CartpoleRobot` Class Initialization
```python
class CartpoleRobot(RobotSupervisorEnv):
    def __init__(self):
        super().__init__()
        self.observation_space = Box(low=np.array([-0.4, -np.inf, -1.3, -np.inf]),
                                     high=np.array([0.4, np.inf, 1.3, np.inf]),
                                     dtype=np.float64)
        self.action_space = Discrete(2)
```

- **`CartpoleRobot` Class**: Extends `RobotSupervisorEnv` to interact with Webots.
- **`observation_space`**: Defines a continuous observation space where the robot observes four values:
    - Cart’s x-position, x-velocity.
    - Pole’s angle, and angular velocity.
- **`action_space`**: Discrete action space with two actions (left or right force applied to the cart).

### 3. Device Setup
```python
self.robot = self.getSelf()
self.position_sensor = self.getDevice("polePosSensor")
self.position_sensor.enable(self.timestep)
self.pole_endpoint = self.getFromDef("POLE_ENDPOINT")
self.wheels = []
for wheel_name in ['wheel1', 'wheel2', 'wheel3', 'wheel4']:
    wheel = self.getDevice(wheel_name)
    wheel.setPosition(float('inf'))
    wheel.setVelocity(0.0)
    self.wheels.append(wheel)
self.steps_per_episode = 200
self.episode_score = 0
self.episode_score_list = []
```

- **Devices**: Configures Webots devices, including a position sensor to monitor the pole angle and wheels to control cart movement.
- **Steps and Scores**: `steps_per_episode` caps each episode at 200 steps, and `episode_score_list` stores scores for performance tracking.

### 4. Observation and Normalization
```python
def get_observations(self):
    cart_position = normalize_to_range(self.robot.getPosition()[0], -0.4, 0.4, -1.0, 1.0)
    cart_velocity = normalize_to_range(self.robot.getVelocity()[0], -0.2, 0.2, -1.0, 1.0, clip=True)
    pole_angle = normalize_to_range(self.position_sensor.getValue(), -0.23, 0.23, -1.0, 1.0, clip=True)
    endpoint_velocity = normalize_to_range(self.pole_endpoint.getVelocity()[4], -1.5, 1.5, -1.0, 1.0, clip=True)
    return [cart_position, cart_velocity, pole_angle, endpoint_velocity]
```

The `get_observations` method normalizes each component to a range of -1 to 1:
- **Cart Position**: Clipped between -0.4 and 0.4 on the x-axis.
- **Cart Velocity**: Velocity on the x-axis is capped within -0.2 and 0.2.
- **Pole Angle**: Constrained within -0.23 and 0.23 radians (about ±13.18 degrees).
- **Endpoint Velocity**: Angular velocity of the pole, limited to ±1.5.

### 5. Reward and Termination Conditions
```python
def get_reward(self, action=None):
    return 1
```

- **Reward**: Constant reward of +1 per step as long as the episode continues, encouraging the agent to keep the pole balanced.

```python
def is_done(self):
    if self.episode_score > 195.0:
        return True
    pole_angle = round(self.position_sensor.getValue(), 2)
    if abs(pole_angle) > 0.261799388:
        return True
    cart_position = round(self.robot.getPosition()[0], 2)
    if abs(cart_position) > 0.39:
        return True
    return False
```

- **Termination**:
    - If the episode score surpasses 195, the episode ends.
    - If the pole angle exceeds ±15 degrees, or the cart position exceeds ±0.39 on the x-axis, the episode also ends.

```python
def solved(self):
    if len(self.episode_score_list) > 100:
        if np.mean(self.episode_score_list[-100:]) > 195.0:
            return True
    return False
```

- **Solved Condition**: The environment is considered solved if the average score of the last 100 episodes exceeds 195, indicating stable balancing.

### 6. Action Application
```python
def apply_action(self, action):
    action = int(action[0])
    motor_speed = 5.0 if action == 0 else -5.0
    for i in range(len(self.wheels)):
        self.wheels[i].setPosition(float('inf'))
        self.wheels[i].setVelocity(motor_speed)
```

- **Action Mapping**: Applies a positive or negative speed to the wheels based on the chosen action, driving the cart left or right.

### 7. Training Loop
```python
env = CartpoleRobot()
agent = PPOAgent(number_of_inputs=env.observation_space.shape[0], number_of_actor_outputs=env.action_space.n)
solved = False
episode_count = 0
episode_limit = 2000
while not solved and episode_count < episode_limit:
    observation = env.reset()
    env.episode_score = 0
    for step in range(env.steps_per_episode):
        selected_action, action_prob = agent.work(observation, type_="selectAction")
        new_observation, reward, done, info = env.step([selected_action])
        trans = Transition(observation, selected_action, action_prob, reward, new_observation)
        agent.store_transition(trans)
        if done:
            env.episode_score_list.append(env.episode_score)
            agent.train_step(batch_size=step + 1)
            solved = env.solved()
            break
        env.episode_score += reward
        observation = new_observation
    print("Episode #", episode_count, "score:", env.episode_score)
    episode_count += 1
```

- **Training Loop**: 
    - The agent interacts with the environment, selecting actions, receiving rewards, and storing transitions.
    - At the end of each episode, the agent trains with the collected transitions, and the environment checks if the task is solved.

### 8. Testing Mode
```python
if not solved:
    print("Task is not solved, deploying agent for testing...")
elif solved:
    print("Task is solved, deploying agent for testing...")
observation = env.reset()
env.episode_score = 0.0
while True:
    selected_action, action_prob = agent.work(observation, type_="selectActionMax")
    observation, _, done, _ = env.step([selected_action])
    if done:
        observation = env.reset()
```

Once training is complete, the agent operates in "testing" mode, continually attempting to balance the pole without exploring alternative actions, allowing it to demonstrate its learned policy.

### 9. Error Message
The terminal output shows that by the 907th episode, the agent consistently achieves a maximum reward of 196, which is considered solved. However, there are Webots-related errors indicating missing dependencies and declaration issues. These errors can be resolved by following the instructions to update the Webots project to R2023b and adding `EXTERNPROTO` declarations for `TexturedBackground`, `TexturedBackgroundLight`, and `RectangleArena` as suggested in the error message.

### Summary
This script effectively implements PPO to train an agent to balance a cart-pole system in Webots, using `deepbots` and a structured reward system. The training loop monitors the agent’s performance, adjusting until the environment is considered solved, after which the agent is deployed in test mode.
