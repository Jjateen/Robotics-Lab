# Gantry Robot Control Using Bluetooth Gamepad

<p align="center">
  <img src="./demo%202.png" alt="gantry">
</p>

This project demonstrates controlling a gantry robot using a Bluetooth gamepad through Webots and Python. The gantry robot is designed to move along rails, lift objects, and manipulate a turret and gripper. The gamepad inputs control various motors and joints, enabling complex robot movements.

## Setup and Requirements

- **Webots**: Ensure you have the Webots simulator installed.
- **Pygame**: Install Pygame for handling gamepad input:
  ```bash
  pip install pygame
  ```
- **Bluetooth Gamepad**: A compatible Bluetooth gamepad must be connected and configured.

## Code Overview

### Constants and Initialization

- **CONTROL_STEP**: Defines the control loop frequency.
- **GANTRY_SPEED_MULTIPLIER**: Adjusts the speed of the gantry's left-right movement.
- **Gripper Positions**: Defines open and close positions for the gripper.

### Webots and Motor Setup

1. **Webots Robot Object**: Initializes the Webots simulation environment.
2. **Motors**: Each motor controlling the gantry, bridge, lift, turret, and gripper is configured for velocity control.

### Gamepad Initialization

- Uses `pygame` to detect and initialize the gamepad. If no gamepad is detected, the program will exit.
- Handles joystick deadzone adjustments to prevent unintended movements.

### Gamepad Input Handling

- **Left Joystick**: Controls the gantry movement (left/right).
- **Right Joystick**: Controls the bridge (left/right) and lift (up/down) movements.
- **Turret Rotation**: Controlled by the RB and LB buttons to rotate clockwise or counter-clockwise.
- **Gripper Control**: LT and RT buttons open and close the gripper.

### Main Loop

- Continuously polls the gamepad inputs and updates the robot's motors accordingly.

## How to Run

1. Open Webots and load the gantry robot world.
2. Connect your Bluetooth gamepad and ensure it is detected by your system.
3. Run the Python script:
   ```bash
   python gantry_gamepad.py
   ```
4. The gamepad inputs will be mapped to the robot's movements in the simulation.

## Script Explanation

### Python Code

```python
import pygame
import sys
from controller import Robot, Motor

# Constants
CONTROL_STEP = 64
JOYSTICK_DEADZONE = 0.1
GANTRY_SPEED_MULTIPLIER = 5.0
GRIPPER_OPEN_POSITION = 0.00
GRIPPER_CLOSE_POSITION = 0.1
MAX_BRIDGE_VELOCITY = 1.0

# Webots initialization
robot = Robot()

# Motor setup
motors = {
    "left_wheel1": robot.getDevice("wheel1_motor"),
    "left_wheel2": robot.getDevice("wheel2_motor"),
    "right_wheel3": robot.getDevice("wheel3_motor"),
    "right_wheel4": robot.getDevice("wheel4_motor"),
    "bridge_motor": robot.getDevice("bridge_motor"),
    "lift_motor": robot.getDevice("lift_motor"),
    "turret_motor": robot.getDevice("turret_motor")
}
gripper = {
    "grip_motor1": robot.getDevice("grip_motor1"),
    "grip_motor2": robot.getDevice("grip_motor2")
}

# Set initial motor states
for motor in motors.values():
    motor.setPosition(float('inf'))
    motor.setVelocity(0.0)

# Gamepad setup
def initialize_gamepad():
    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        print("No joystick detected. Please connect a gamepad.")
        pygame.quit()
        sys.exit()
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick

# Joystick input processing
def apply_deadzone(value):
    return value if abs(value) >= JOYSTICK_DEADZONE else 0.0

# Mapping gamepad inputs to motors
def handle_gamepad_input(joystick):
    pygame.event.pump()
    # Gantry movement
    left_x = apply_deadzone(joystick.get_axis(0)) * GANTRY_SPEED_MULTIPLIER * -1
    motors["left_wheel1"].setVelocity(left_x)
    motors["left_wheel2"].setVelocity(left_x)
    motors["right_wheel3"].setVelocity(left_x)
    motors["right_wheel4"].setVelocity(left_x)

    # Gripper bridge control
    right_x = apply_deadzone(joystick.get_axis(2)) * GANTRY_SPEED_MULTIPLIER * -1
    if abs(right_x) > MAX_BRIDGE_VELOCITY:
        right_x = MAX_BRIDGE_VELOCITY * (1 if right_x > 0 else -1)
    motors["bridge_motor"].setVelocity(right_x)

    # Lift control
    right_y = apply_deadzone(joystick.get_axis(3))
    motors["lift_motor"].setVelocity(right_y)

    # Gripper open/close
    lt = joystick.get_button(1)
    rt = joystick.get_button(0)
    if lt:
        gripper["grip_motor1"].setPosition(GRIPPER_OPEN_POSITION)
        gripper["grip_motor2"].setPosition(GRIPPER_OPEN_POSITION)
    elif rt:
        gripper["grip_motor1"].setPosition(GRIPPER_CLOSE_POSITION)
        gripper["grip_motor2"].setPosition(GRIPPER_CLOSE_POSITION)

    # Turret rotation
    if joystick.get_button(5):
        motors["turret_motor"].setVelocity(1.0)
    elif joystick.get_button(4):
        motors["turret_motor"].setVelocity(-1.0)
    else:
        motors["turret_motor"].setVelocity(0)

# Main loop
def main():
    joystick = initialize_gamepad()
    while robot.step(CONTROL_STEP) != -1:
        handle_gamepad_input(joystick)

if __name__ == "__main__":
    main()
```

### Webots World File

The Webots world file defines the gantry robot, its environment, and object setups necessary for the simulation.

**World Information**
- A rail system supporting the gantry and mechanisms for lifting and gripping.
- Custom appearance and background settings.
- Multiple platforms and boxes to interact with.

---
