clc;
clear all;
close all;

% Define the robot links (5-DOF robot)
L(1) = Link([0 675 350 -pi/2], 'standard');
L(2) = Link([0 0 1350 0], 'standard');
L(3) = Link([0 0 1220 0], 'standard');
L(4) = Link([0 0 280 pi/2], 'standard');
L(5) = Link([0 0 0 0], 'standard');

% Create the SerialLink object for the 5-DOF robot
KR = SerialLink(L);

% Define the final joint configuration
qf = [pi/2 -pi/6 -pi/4 pi/2 0];
Tf = KR.fkine(qf);  % Compute forward kinematics to get the target pose

% Define the initial joint configuration (try a different one to avoid singularity)
q0 = [pi/6 0 -pi/6 0 pi/6];

M = [1 1 1 1 1 0];
% Inverse kinematics to solve for joint angles
q = KR.ikine(Tf, q0, M);

% Define the time vector for trajectory
t = 0:0.15:3;

% Generate a joint trajectory between q0 and qf
Q = jtraj(q0, qf, t);

% Extract and store the trajectory in Cartesian space
xx = zeros(length(t), 1);
yy = zeros(length(t), 1);
zz = zeros(length(t), 1);
for i = 1:length(t)
    T = KR.fkine(Q(i, :));  % Forward kinematics for each step
    trs = transl(T);        % Extract the translation component
    xx(i) = trs(1);
    yy(i) = trs(2);
    zz(i) = trs(3);
end

% Plot the robot movement
KR.plot(Q);

% Plot the trajectory in Cartesian space using plot3
hold on;
plot3(xx, yy, zz, 'r', 'LineWidth', 2);  % Plot the trajectory in 3D space

% Format the plot
view(3);
axis vis3d;
xlabel('X-axis');
ylabel('Y-axis');
zlabel('Z-axis');
grid on;
