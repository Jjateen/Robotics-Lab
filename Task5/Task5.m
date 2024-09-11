
clc
clear all
close all

% Define the DH parameters for a 6 DOF robot (5 revolute, 1 prismatic)
L1 = Link([0 0 0 0], 'standard', 'modified');  % Prismatic joint (1st joint)
L1.sigma = 1;  % Set the 1st joint as prismatic (sigma = 1 for prismatic joints)
L1.qlim = [0 2];  % Limit the displacement for the prismatic joint

L2 = Link([0 1 1 pi/2], 'standard');  % Revolute joint
L3 = Link([0 0 1 0], 'standard');  % Revolute joint
L4 = Link([0 0 0 pi/2], 'standard');  % Revolute joint
L5 = Link([0 0 0 -pi/2], 'standard');  % Revolute joint
L6 = Link([0 0 0 0], 'standard', 'modified');  % Prismatic joint

L6.sigma = 1 ;
% Set the prismatic joint's motion (q6 will define the displacement)
L6.qlim = [0 2];  % Limit the prismatic joint displacement (example: 0 to 2 units)
L1.qlim = [0 2];
% Create the SerialLink robot
R = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6DOF_Robot');

% Define the initial joint configuration for the plot
q_init = [0, 0, 0, 0, 0, 0];  % Starting joint angles

% Plot the robot
figure;  % Create a new figure window
R.plot(q_init);  % Plot the robot

% Ensure 3D view
view(3);          % Set the view to 3D
axis vis3d;       % Maintain aspect ratio and enable 3D view
grid on;          % Show grid

% Open the interactive control panel
R.teach();        % Open the interactive control panel
