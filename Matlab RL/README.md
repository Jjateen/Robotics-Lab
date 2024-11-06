![Bipedal Bot on a curved track](./RL1.png)
![Ball Balancing Bot](./RL2.png)

# Train SAC Agent for Ball Balance Control

This example demonstrates how to train a Soft Actor-Critic (SAC) reinforcement learning agent to control a Kinova Gen3 robot arm for a ball-balancing task. 

## Introduction

The task involves a Kinova Gen3 robot arm with seven degrees of freedom (DOF). The goal is to balance a ping pong ball at the center of a flat plate attached to the robot's gripper. Only the final two joints of the robot arm are actuated, enabling motion in the pitch and roll axes, as illustrated below. The other joints are fixed.

### Simulink Model

The Simulink® model contains a Kinova Ball Balance subsystem connected to an RL Agent block. The agent applies an action to the robot subsystem and receives the resulting observation, reward, and is-done signals.

```matlab
open_system("rlKinovaBallBalance")
```

You can explore the Kinova Ball Balance subsystem here:

```matlab
open_system("rlKinovaBallBalance/Kinova Ball Balance")
```

### System Components

- The physical components (manipulator, ball, and plate) are modeled using Simscape™ Multibody™.
- The plate is attached to the robot's end effector.
- The ball can move freely in six degrees of freedom.
- The Spatial Contact Force block models the contact forces between the ball and the plate.
- The control inputs to the robot are torque signals for the actuated joints.

To view a 3D animation of the manipulator, set the Visualization parameter to *3D Mesh* in the Mechanics Explorer. If you do not have the Robotics System Toolbox Robot Library Data support package, set the Visualization parameter to *None*. You can install the package using Add-On Explorer. 

### Setting Parameters

Run the `kinova_params` script to initialize parameters for the example. If you have the required support package installed, this script will also add the necessary mesh files to your MATLAB® path.

```matlab
kinova_params
```

## Defining the Environment

The ball-balancing environment is defined with:

- **Observations**: A 22-element vector containing:
  - Joint positions (sine and cosine of angles) and velocities
  - Ball positions (x and y distances from the plate center) and velocities
  - Plate orientation (quaternions) and velocities
  - Joint torques, ball radius, and mass
- **Actions**: Normalized torque values for the two actuated joints
- **Sample time**: `Ts = 0.01`, **Simulation time**: `Tf = 10`

The simulation terminates when the ball falls off the plate.

### Reward Function

The reward function \( R_t \) at time step \( t \) is defined as:

![Reward Function](https://latex.codecogs.com/png.image?\dpi{120}\color{White}R_t&space;=&space;R_{ball}&space;-&space;R_{plate}&space;-&space;R_{control})

where:

- ![Reward for Ball](https://latex.codecogs.com/png.image?\dpi{120}\color{White}R_{ball}): Reward for the ball moving closer to the plate center
- ![Penalty for Plate](https://latex.codecogs.com/png.image?\dpi{120}\color{White}R_{plate}): Penalty for plate orientation
- ![Control Effort Penalty](https://latex.codecogs.com/png.image?\dpi{120}\color{White}R_{control}): Penalty for control effort
- ![Angles](https://latex.codecogs.com/png.image?\dpi{120}\color{White}\phi,&space;\theta,&space;\psi): Plate roll, pitch, and yaw angles in radians
- ![Joint Torques](https://latex.codecogs.com/png.image?\dpi{120}\color{White}\tau_1,&space;\tau_2): Joint torques

## Setting Up the Environment Interface

Create the observation and action specifications:

```matlab
nObs = 22; % Number of observation dimensions
nAct = 2;  % Number of action dimensions

obsInfo = rlNumericSpec([nObs 1]);
actInfo = rlNumericSpec([nAct 1]);
actInfo.LowerLimit = -1;
actInfo.UpperLimit = 1;
```

Create the Simulink environment interface:

```matlab
mdl = "rlKinovaBallBalance";
blk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl, blk, obsInfo, actInfo);
env.ResetFcn = @kinovaResetFcn;
```

## Creating the Agent

The SAC agent uses two Q-value function approximators (critics) and a continuous Gaussian actor.

### Critic Neural Network

The critic network estimates the value of the policy:

```matlab
observationPath = [
    featureInputLayer(nObs, Name="observation")
    concatenationLayer(1, 2, Name="concat")
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer
    fullyConnectedLayer(32)
    reluLayer
    fullyConnectedLayer(1, Name="QValueOutLyr")
];
actionPath = featureInputLayer(nAct, Name="action");

criticNet = dlnetwork;
criticNet = addLayers(criticNet, observationPath);
criticNet = addLayers(criticNet, actionPath);
criticNet = connectLayers(criticNet, "action", "concat/in2");
```

### Actor Neural Network

The actor network outputs a Gaussian-distributed action:

```matlab
commonPath = [
    featureInputLayer(nObs, Name="observation")
    fullyConnectedLayer(128)
    reluLayer
    fullyConnectedLayer(64)
    reluLayer(Name="commonPath")
];
meanPath = [
    fullyConnectedLayer(32, Name="meanFC")
    reluLayer
    fullyConnectedLayer(nAct, Name="actionMean")
];
stdPath = [
    fullyConnectedLayer(nAct, Name="stdFC")
    reluLayer
    softplusLayer(Name="actionStd")
];

actorNet = dlnetwork;
actorNet = addLayers(actorNet, commonPath);
actorNet = addLayers(actorNet, meanPath);
actorNet = addLayers(actorNet, stdPath);
actorNet = connectLayers(actorNet, "commonPath", "meanFC/in");
actorNet = connectLayers(actorNet, "commonPath", "stdFC/in");
```

### Creating the SAC Agent

```matlab
agentOpts = rlSACAgentOptions( ...
    SampleTime = Ts, ...
    TargetSmoothFactor = 1e-3, ...
    ExperienceBufferLength = 1e6, ...
    MiniBatchSize = 256, ...
    NumWarmStartSteps = 256 * 10, ...
    DiscountFactor = 0.99);
```

Initialize and create the agent:

```matlab
agent = rlSACAgent(actor, [critic1, critic2], agentOpts);
```

## Training the Agent

Configure the training options:

```matlab
trainOpts = rlTrainingOptions( ...
    MaxEpisodes = 6000, ...
    MaxStepsPerEpisode = floor(Tf/Ts), ...
    ScoreAveragingWindowLength = 100, ...
    Plots = "training-progress", ...
    SimulationStorageType = "file", ...
    StopTrainingCriteria = "EvaluationStatistic", ...
    StopTrainingValue = 700, ...
    UseParallel = false);
```

To train the agent:

```matlab
doTraining = false;
if doTraining
    evaluator = rlEvaluator(EvaluationFrequency = 25, NumEpisodes = 5);
    trainResult = train(agent, env, trainOpts, Logger = logger, Evaluator = evaluator);
else
    load("kinovaBallBalanceAgent.mat");
end
```

## Simulating the Trained Agent

Set initial conditions and run the simulation:

```matlab
userSpecifiedConditions = true;
simOpts = rlSimulationOptions(MaxSteps = floor(Tf/Ts));
set_param(mdl, SimMechanicsOpenEditorOnUpdate = "on");
doViz = true;
agent.UseExplorationPolicy = false;
experiences = sim(agent, env, simOpts);
```

Visualize the ball trajectory:

```matlab
fig = animatedPath(experiences);
```

## Data Logging Functions

```matlab
function dataToLog = logAgentLearnData(data)
    dataToLog.ActorLoss = data.ActorLoss;
    dataToLog.CriticLoss = data.CriticLoss;
end

function dataToLog = logEpisodeData(data, doViz)
    dataToLog.Experience = data.Experience;
    if doViz
        animatedPath(data.Experience);
    end
end
```

---

**Copyright 2021-2024 The MathWorks, Inc.**
