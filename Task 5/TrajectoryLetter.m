clc;
clear all;
close all;

% Define the DH parameters for a 6 DOF robot (5 revolute, 1 prismatic)
L1 = Link([0 0 0 0], 'standard', 'modified');  % Prismatic joint (1st joint)
L1.sigma = 1;  % Set the 1st joint as prismatic (sigma = 1 for prismatic joints)
L1.qlim = [0 2];  % Limit the displacement for the prismatic joint

L2 = Link([0 1 1 pi/2], 'standard');  % Revolute joint
L3 = Link([0 0 1 0], 'standard');  % Revolute joint
L4 = Link([0 0 0 pi/2], 'standard');  % Revolute joint
L5 = Link([0 0 0 -pi/2], 'standard');  % Revolute joint
L6 = Link([0 0 0 0], 'standard', 'modified');  % Prismatic joint

L6.sigma = 1;  % Set the prismatic joint as prismatic
L6.qlim = [0 2];  % Limit the prismatic joint displacement (example: 0 to 2 units)

% Create the SerialLink robot
R = SerialLink([L1 L2 L3 L4 L5 L6], 'name', '6DOF_Robot');

% Define the trajectory for the letter "J" in 3D space
% "J" trajectory in XYZ coordinates (in meters)
J_traj = [...
    0.5, 0, 1;    % Top of the "J"
    0.5, 0, 0.8;  % Downwards line
    0.5, 0, 0.6;
    0.5, 0, 0.4;
    0.5, 0, 0.2;
    0.4, 0, 0.1;  % Curve of the "J"
    0.3, 0, 0;
    0.2, 0, 0;
    0.1, 0, 0;
    0, 0, 0;      % Bottom of the "J"
];

% Initialize joint configurations
q_init = [0, 0, 0, 0, 0, 0];  % Initial joint angles
q_trajectory = [];

% Set the mask to prioritize XYZ position control (translation)
mask = [1 1 1 0 0 0];  % Only care about position (no orientation control)

% Loop over each point in the J trajectory and calculate inverse kinematics
for i = 1:size(J_traj, 1)
    T = transl(J_traj(i, :));  % Create the translation matrix for the point
    q = R.ikine(T, q_init, mask);  % Inverse kinematics for the given point
    q_trajectory = [q_trajectory; q];  % Store the joint angles
    q_init = q;  % Update the initial position for the next point
end

% Plot the robot in the initial configuration
figure;
R.plot(q_trajectory(1, :));
view(3);  % Set the view to 3D
axis vis3d;
grid on;

% Animate the robot to draw the letter "J"
for i = 1:size(q_trajectory, 1)
    R.plot(q_trajectory(i, :));  % Plot each joint configuration
    pause(0.2);  % Add a small pause to slow down the animation
end

% Perform trajectory analysis
% Plot each joint's position over time
figure;
subplot(3,2,1);
plot(q_trajectory(:, 1));
title('Joint 1 Position');
xlabel('Trajectory Step');
ylabel('Joint Angle (rad)');

subplot(3,2,2);
plot(q_trajectory(:, 2));
title('Joint 2 Position');
xlabel('Trajectory Step');
ylabel('Joint Angle (rad)');

subplot(3,2,3);
plot(q_trajectory(:, 3));
title('Joint 3 Position');
xlabel('Trajectory Step');
ylabel('Joint Angle (rad)');

subplot(3,2,4);
plot(q_trajectory(:, 4));
title('Joint 4 Position');
xlabel('Trajectory Step');
ylabel('Joint Angle (rad)');

subplot(3,2,5);
plot(q_trajectory(:, 5));
title('Joint 5 Position');
xlabel('Trajectory Step');
ylabel('Joint Angle (rad)');

subplot(3,2,6);
plot(q_trajectory(:, 6));
title('Joint 6 Position');
xlabel('Trajectory Step');
ylabel('Joint Displacement (m)');  % Joint 6 is prismatic

% Open the interactive control panel for further manual control
R.teach();