function trainSweep 
% Walking Robot -- function to iterate trainings over various random seeds.
% 
% Copyright 2024 The MathWorks, Inc.

was = pwd;
cd("..");
cln1 = onCleanup(@() cd(was));

%% generate the training variations for each agent type
overwrite = false;
SEEDS = (1:5);
ATYPES = ["TD3","DDPG"];

[A,S] = ndgrid(ATYPES,SEEDS);

dname = A + "Agent";
F = fullfile(dname,"run" + S + ".mat");
ALREADY_RUN = isfile(F);

ud = unique(dname);
for i = 1:numel(ud)
    [~,~] = mkdir(ud(i));
end

% remove the savedSims directory
[~] = rmdir("savedSims","s");

%% setup the environment and training options
evalin("base","robotParametersRL;");
Ts = evalin("base","Ts");
Tf = evalin("base","Tf");
h = evalin("base","h");
upper_leg_length = evalin("base","upper_leg_length");
lower_leg_length = evalin("base","lower_leg_length");

numObs = 29;
obsInfo = rlNumericSpec([numObs 1]);
obsInfo.Name = "observations";

numAct = 6;
actInfo = rlNumericSpec([numAct 1],LowerLimit=-1,UpperLimit=1);
actInfo.Name = "foot_torque";

mdl = "rlWalkingBipedRobot";
blk = mdl + "/RL Agent";
env = rlSimulinkEnv(mdl,blk,obsInfo,actInfo);
env.ResetFcn = @(in) walkerResetFcn(in, ...
    upper_leg_length/100, ...
    lower_leg_length/100, ...
    h/100);

maxEpisodes = 3000;
maxSteps = floor(Tf/Ts);
trainOpts = rlTrainingOptions(...
    MaxEpisodes=maxEpisodes,...
    MaxStepsPerEpisode=maxSteps,...
    ScoreAveragingWindowLength=250,...
    Verbose=false,...
    Plots="training-progress",...
    StopTrainingCriteria="EpisodeCount",...
    StopTrainingValue=maxEpisodes,...
    SimulationStorageType="file");
trainOpts.UseParallel = true;
trainOpts.ParallelizationOptions.Mode = "async";

%% train for each variation
for train_id = 1:numel(A)
    if ALREADY_RUN(train_id) && ~overwrite
        continue;
    end

    rng(S(train_id));
    atype = A(train_id);
    fname = F(train_id);

    if strcmp(atype,"DDPG")
        agent = createDDPGAgent(numObs,obsInfo,numAct,actInfo,Ts);
    else
        agent = createTD3Agent(numObs,obsInfo,numAct,actInfo,Ts);
    end

    agent.AgentOptions.NumEpoch = 3;
    agent.AgentOptions.MaxMiniBatchPerEpoch = 100;
    agent.AgentOptions.LearningFrequency = -1;
    agent.AgentOptions.PolicyUpdateFrequency = 1;
    agent.AgentOptions.TargetUpdateFrequency = 1;

    % Train the agent.
    if isempty(gcp("nocreate"))
        parpool(4);
    end
    trainingStats = train(agent,env,trainOpts);

    % save
    save(fname,"agent","trainingStats");
end
