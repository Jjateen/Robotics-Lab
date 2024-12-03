clc
clear all
close all

mdl_puma560

% Step 1: Define path points for the letter "J"
% Horizontal line from (0.35, 0.4) to (0.65, 0.4)
% Vertical line from (0.5, 0.4) to (0.5, 0.0)
% Semicircle with diameter from (0.5, 0.0) to (0.3, 0.0)

% Semicircle parameters
theta = linspace(0, pi, 50);  % Angle range for the semicircle
radius = 0.1;
semicircle_x = radius * cos(theta) + 0.4;  % Shift center to (0.4, 0)
semicircle_y = radius * sin(theta) * -1;

% Path points (keeping z = 0 throughout)
path = [ 
    0.35 0.4 0;   % Start of horizontal line
    0.65 0.4 0;   % End of horizontal line
    0.5 0.4 0;   % Start of vertical line
    0.5 0.0 0;   % End of vertical line (start of semicircle)
    [semicircle_x', semicircle_y', zeros(length(semicircle_x), 1)]  % Semicircle points
];

% Reduce the velocity for a smoother and slower path
velocities = [0.3 0.3 0.3];   % XYZ velocities (slower than before)

% Define initial coordinates for mstraj
initial_position = path(1, :);  % Starting at the first path point

% Generate trajectory points using mstraj (reduce sample interval to get more points)
p = mstraj(path, velocities, [], initial_position, 0.05, 0.05);  % Smaller interval for higher resolution

% Apply scaling and translation to get final coordinates (writable size)
Tp = transl(p);

% Set the tool orientation (rotate hand along x-axis by 180 degrees)
p560.tool = trotx(pi); 

% Inverse kinematics to find the joint angles for the trajectory
q = p560.ikine6s(Tp);  

% Plot the robot's movement
p560.plot(q);  

% Create a figure to plot the end-effector trajectory
hold on
for i = 1:size(q, 1)
    % Compute forward kinematics for each joint configuration
    T = p560.fkine(q(i, :)); 
    
    % Extract end-effector position (x, y, z)
    position = T(1:3, 4);
    
    % Plot the end-effector position at this time step
    plot3(position(1), position(2), position(3), 'r.', 'MarkerSize', 15);
end

hold off
view(3);
axis vis3d;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End-Effector Path for Letter J');
grid on;