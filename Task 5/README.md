## About

This repository contains a copy of Peter Corke's MATLAB Robotics Toolbox. The scripts within are utilized for various lab tasks.

![Task5a](./Task%205a.png)
![Task5b](./Task%205b.png)
![Task5c](./Task%205c.png)

This MATLAB script simulates a PUMA 560 robotic arm writing the letter "J" by following a series of predefined path points. It requires the Robotics Vision Toolbox (RVC Toolbox), so before running the script, initialize the toolbox by typing `startup_rvc` in the command window.

### 1. Initial Setup
```matlab
clc;
clear all;
close all;

mdl_puma560;  % Load the PUMA 560 robot model
```
The script begins by clearing the workspace, closing all figures, and loading the PUMA 560 model provided by the RVC Toolbox.

### 2. Define Path Points for the Letter "J"
To create the "J" shape, the script defines three main path segments:
1. **Horizontal line** from `(0.35, 0.4)` to `(0.65, 0.4)`.
2. **Vertical line** from `(0.5, 0.4)` to `(0.5, 0.0)`.
3. **Semicircle** with a diameter along the x-axis to form the lower curve of the "J".

The semicircle is generated using polar coordinates:
```matlab
theta = linspace(0, pi, 50);  % Angle range for the semicircle
radius = 0.1;
semicircle_x = radius * cos(theta) + 0.4;  % Shift center to (0.4, 0)
semicircle_y = radius * sin(theta) * -1;  % Invert y-axis for downward curve
```

### 3. Combine Path Points
The complete path for the letter "J" is created by joining the segments in the `path` matrix:
```matlab
path = [ 
    0.35 0.4 0;   % Start of horizontal line
    0.65 0.4 0;   % End of horizontal line
    0.5 0.4 0;    % Start of vertical line
    0.5 0.0 0;    % End of vertical line (start of semicircle)
    [semicircle_x', semicircle_y', zeros(length(semicircle_x), 1)]  % Semicircle points
];
```
This matrix contains each path point in the sequence needed to trace the "J."

### 4. Generate Trajectory Points with `mstraj`
The script uses `mstraj` to generate smooth trajectory points that the robot follows. By setting smaller intervals, it increases the path resolution for smoother movement.

```matlab
velocities = [0.3 0.3 0.3];  % XYZ velocities for slower motion
initial_position = path(1, :);  % Start point

% Generate trajectory points
p = mstraj(path, velocities, [], initial_position, 0.05, 0.05);
```

### 5. Define Tool Orientation and Calculate Joint Angles
The end-effector is rotated 180° along the x-axis to make it point downward, which is suitable for writing. The script then uses inverse kinematics to calculate the joint angles required to achieve each point in the trajectory.

```matlab
Tp = transl(p);  % Convert trajectory points to transformations
p560.tool = trotx(pi);  % Rotate tool 180 degrees
q = p560.ikine6s(Tp);  % Calculate joint angles using inverse kinematics
```

### 6. Plot the Robot's Movement
Using `p560.plot(q)`, the robot's movement along the path is animated, showing the robot following the "J" shape in 3D space.

### 7. Plot the End-Effector Path
The script also plots the actual trajectory of the end-effector (the robot's "pen" position) as red dots. This is achieved by calculating forward kinematics for each joint configuration.

```matlab
hold on
for i = 1:size(q, 1)
    T = p560.fkine(q(i, :));  % Forward kinematics for each joint configuration
    position = T(1:3, 4);     % Extract end-effector position (x, y, z)
    plot3(position(1), position(2), position(3), 'r.', 'MarkerSize', 15);
end
```

### Final Plot Settings
The final 3D plot provides a complete visualization of the "J" shape traced by the end-effector, with axes labels and a title.

```matlab
view(3);
axis vis3d;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('End-Effector Path for Letter J');
grid on;
```

### Summary
This simulation demonstrates the PUMA 560 robotic arm tracing a "J" in 3D space by following a defined path of points. The RVC Toolbox provides the necessary functions for creating the robot model, calculating inverse and forward kinematics, and visualizing the motion.
