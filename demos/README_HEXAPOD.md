# Hexapod Robot Controlled via Bluetooth Gamepad

<p align="center">
  <img src="./demo%201.png" alt="hexapod">
</p>

This project involves the implementation of a hexapod robot simulation controlled through a Bluetooth gamepad. The hexapod employs a biologically inspired gait for forward, backward, and turning movements. The project is designed and tested using the Webots robotics simulation platform and a Python script integrating Pygame for gamepad control.

---

## Overview

The hexapod robot has 12 motors:
- **6 Hip Motors**: Responsible for controlling the forward and backward motion of the legs.
- **6 Knee Motors**: Control the up and down motion of the legs for walking and lifting.

The robot is controlled via a Bluetooth gamepad, where joystick input determines the direction of motion:
- **Left joystick Y-axis**: Controls forward and backward movement.
- **Left joystick X-axis**: Controls left and right turning.

A tripod gait algorithm is used to achieve stable and efficient movement.

---

## Project Files

### 1. **hexapod.py**
This is the main Python script that handles:
- Initialization of the Webots simulation.
- Control of the hexapod's motors.
- Integration of gamepad input using Pygame.
- Execution of different gaits based on joystick commands.

### 2. **hexapod.wbt**
A Webots world file that defines the environment for the hexapod simulation. This file sets up a basic simulation space and configures the hexapod robot's components.

---

## Installation and Setup

### Prerequisites
- **Webots**: Install Webots R2023b or a compatible version from the [Webots website](https://cyberbotics.com).
- **Python**: Make sure Python 3.8+ is installed.
- **Pygame**: Install using the following command:
  ```bash
  pip install pygame
  ```

### Configuration
1. Open Webots and load the `hexapod.wbt` world file.
2. Run the `hexapod.py` script in your Python environment.

---

## Detailed Code Explanation

### Constants and Variables
- **TIME_STEP**: Simulation time step, set to 16 ms for smooth control.
- **NUM_MOTORS**: The total number of motors in the hexapod (12 in total).
- **FRONT, BACK, HI, LO**: Constants for setting hip and knee motor positions during gaits.
- **DEAD_ZONE**: A threshold to filter out minor joystick movements and prevent unintentional commands.
- **AXIS_LEFT_X, AXIS_LEFT_Y**: Joystick axes for controlling movement.

### Initialization
The `initialize_gamepad()` function sets up the Pygame library and initializes the Bluetooth gamepad. The program will terminate if no gamepad is detected.

### Motor Setup
The script initializes all motors using the Webots API:
- Motors are configured to use velocity control.
- The initial velocity is set to zero to keep the robot stationary.

### Gait Control
The hexapod uses a tripod gait for movement:
- **Tripod Gait**: The legs are divided into two groups, each consisting of three legs. One group moves forward while the other provides support.
- **State Machine**: The robot cycles through six gait states to produce a continuous walking motion.

#### Movement Logic
1. **Forward/Backward Movement**:
   - The hip motors use predefined sequences (`pos_hip_forward` and `pos_hip_backward`) to move the legs.
   - Knee motors are adjusted to lift and lower the legs appropriately.
2. **Turning**:
   - Alternate tripod movements are used to achieve left and right turns.
   - The hip motors are configured for different angles to pivot the robot, while knee motors maintain stability.

### Gamepad Control
- The gamepad's left joystick determines the movement direction.
- `apply_dead_zone()` filters joystick input to avoid unwanted small movements.
- Joystick input is used to update motor positions and speeds dynamically.

---

## Running the Simulation

1. **Launch Webots**: Open the `hexapod.wbt` file.
2. **Run the Python Script**: Execute `hexapod.py` to start controlling the robot with the gamepad.
3. **Gamepad Input**:
   - Push the left joystick forward/backward to move the robot.
   - Move the joystick left/right to turn the robot.

---

## Troubleshooting

- **No Gamepad Detected**: Ensure your Bluetooth gamepad is properly connected and recognized by your system.
- **Laggy Movement**: Adjust `TIME_STEP` or reduce simulation complexity for better performance.
- **Motor Configuration Issues**: Verify that motor names in `MOTOR_NAMES` match those in your Webots simulation.

---

## Future Enhancements

- **Add Obstacle Avoidance**: Implement sensors for autonomous navigation.
- **Refine Gait**: Optimize the tripod gait for smoother and faster movement.
- **Real-World Application**: Test the script on a physical hexapod robot using ROS for real-time control.

---

## References

- [Webots Documentation](https://cyberbotics.com/doc/guide/index)
- [Pygame Documentation](https://www.pygame.org/docs/)

---
