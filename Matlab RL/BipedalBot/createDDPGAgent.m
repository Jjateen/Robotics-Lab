function agent = createDDPGAgent(numObs, obsInfo, numAct, actInfo, Ts)
% Walking Robot -- DDPG Agent Setup Script
% Copyright 2024 The MathWorks, Inc.

%% Create the actor and critic networks using the createNetworks helper function
[criticNetwork,~,actorNetwork] = createNetworks(numObs,numAct);

%% Specify options for the critic and actor representations using rlOptimizerOptions
criticOptions = rlOptimizerOptions('Optimizer','adam','LearnRate',1e-3,... 
                                        'GradientThreshold',1);
actorOptions = rlOptimizerOptions('Optimizer','adam','LearnRate',1e-3,...
                                       'GradientThreshold',1);

%% Create critic and actor representations using specified networks and
% options
critic = rlQValueFunction(criticNetwork,obsInfo,actInfo,'ObservationInputNames','observation','ActionInputNames','action');
actor  = rlContinuousDeterministicActor(actorNetwork,obsInfo,actInfo);

%% Specify DDPG agent options
agentOptions = rlDDPGAgentOptions;
agentOptions.SampleTime = Ts;
agentOptions.DiscountFactor = 0.99;
agentOptions.MiniBatchSize = 256;
agentOptions.ExperienceBufferLength = 1e6;
agentOptions.TargetSmoothFactor = 5e-3;

agentOptions.NumEpoch = 3;
agentOptions.MaxMiniBatchPerEpoch = 100;
agentOptions.LearningFrequency = -1;
agentOptions.PolicyUpdateFrequency = 1;
agentOptions.TargetUpdateFrequency = 1;

agentOptions.NoiseOptions.MeanAttractionConstant = 1;
agentOptions.NoiseOptions.StandardDeviation = 0.1;

agentOptions.ActorOptimizerOptions = actorOptions;
agentOptions.CriticOptimizerOptions = criticOptions;

%% Create agent using specified actor representation, critic representation and agent options
agent = rlDDPGAgent(actor,critic,agentOptions);
