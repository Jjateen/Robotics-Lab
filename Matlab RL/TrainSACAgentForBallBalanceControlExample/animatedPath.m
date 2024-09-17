function fig = animatedPath(experience)
% Animate the ball path on the plate
% experience is the output from sim(agent,env,simOpts)

% Copyright 2021-2022, The MathWorks Inc.

persistent ax hball hpath plate platewidth

% check if the experience is from the output of sim command
fromSimOut = isa(experience(1).Reward, "timeseries");

if fromSimOut
    % experience is from output of sim command
    numObs = size(experience.Observation.obs1.Data,1);
    numSteps = numel(experience.Observation.obs1.Time);
    allObs = reshape(experience.Observation.obs1.Data,numObs,numSteps);
    obs0 = allObs(:,1);
else
    % experience is logged from a data logging function
    obs0 = experience(1).Observation{1};
    numSteps = numel(experience);
    x = [experience.Observation];
    allObs = horzcat(x{:});
end
ballx0 = obs0(7);
bally0 = obs0(8);
radius = obs0(21);

if isempty(ax) || ~isvalid(ax)
    fig = figure("Name","Ball Balance Animation", ...
        "NumberTitle","off", ...
        "MenuBar","none", ...
        "Position",[500,500,300,300]);
    set(gcf,'Visible','on');
    ax = gca(fig);
    hold(ax,'on')
    title(ax,'Ball position on plate')
    xlabel(ax,'X (m)');
    ylabel(ax,'Y (m)');

    plate = evalin('base','plate');
    platewidth = plate.width;

    % plot plate
    rectangle(ax,"Position",platewidth*[-0.5,-0.5,1,1],"FaceColor","c");

    % plot ball path
    hpath = plot(ballx0,bally0,"b.");
    hpath.LineWidth=0.1;

    % plot ball
    hball = rectangle(ax, ...
        "Position", [ballx0,bally0,0,0] + 2*radius*[-0.5,-0.5,1,1], ...
        "Curvature", [1,1], ...
        "FaceColor","r");

    axis(ax,"equal");
    ax.XLim = 1.05 * platewidth * [-1,1];
    ax.YLim = 1.05 * platewidth * [-1,1];
    ax.Visible = true;
else
    fig = ax.Parent;
    hpath.XData = [];
    hpath.YData = [];
end

% Update ball position and path
for idx = 1:numSteps
    obs = allObs(:,idx);
    ballx = obs(7);
    bally = obs(8);
    hpath.XData = [hpath.XData ballx];
    hpath.YData = [hpath.YData bally];
    hball.Position = [ballx,bally,0,0] + 2*radius*[-0.5,-0.5,1,1];
    drawnow();
end
