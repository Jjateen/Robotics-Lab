import pygame
import sys
from controller import Robot, Motor

# Constants
CONTROL_STEP = 64
JOYSTICK_DEADZONE = 0.1
GANTRY_SPEED_MULTIPLIER = 5.0  # Speed multiplier for the gantry movement
GRIPPER_OPEN_POSITION = 0.00   # Gripper open position
GRIPPER_CLOSE_POSITION = 0.1   # Gripper close position
MAX_BRIDGE_VELOCITY = 1.0      # Maximum velocity for the bridge motor

# Webots initialization
robot = Robot()

# Motor initialization
motors = {
    "left_wheel1": robot.getDevice("wheel1_motor"),
    "left_wheel2": robot.getDevice("wheel2_motor"),
    "right_wheel3": robot.getDevice("wheel3_motor"),
    "right_wheel4": robot.getDevice("wheel4_motor"),
    "bridge_motor": robot.getDevice("bridge_motor"),
    "lift_motor": robot.getDevice("lift_motor"),
    "turret_motor": robot.getDevice("turret_motor")  # Added turret motor
}
gripper = { "grip_motor1": robot.getDevice("grip_motor1"),
            "grip_motor2": robot.getDevice("grip_motor2"),
}
# Set motor velocities and initial positions
for motor in motors.values():
    motor.setPosition(float('inf'))  # Allow velocity control
    motor.setVelocity(0.0)

# Pygame gamepad setup
def initialize_gamepad():
    pygame.init()
    pygame.joystick.init()

    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected. Please connect a gamepad.")
        pygame.quit()
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick

# Apply deadzone to joystick input
def apply_deadzone(value):
    return value if abs(value) >= JOYSTICK_DEADZONE else 0.0

# Handling gamepad input and mapping to robot motors
def handle_gamepad_input(joystick):
    pygame.event.pump()

    # Left joystick X-axis (move gantry left/right)
    left_x = apply_deadzone(joystick.get_axis(0)) * GANTRY_SPEED_MULTIPLIER * -1
    motors["left_wheel1"].setVelocity(left_x)
    motors["left_wheel2"].setVelocity(left_x)
    motors["right_wheel3"].setVelocity(left_x)
    motors["right_wheel4"].setVelocity(left_x)

    # Right joystick X-axis (move gripper left/right)
    right_x = apply_deadzone(joystick.get_axis(2)) * GANTRY_SPEED_MULTIPLIER * -1
    if abs(right_x) > MAX_BRIDGE_VELOCITY:
        right_x = MAX_BRIDGE_VELOCITY * (1 if right_x > 0 else -1)
    motors["bridge_motor"].setVelocity(right_x)

    # Right joystick Y-axis (move gripper up/down)
    right_y = apply_deadzone(joystick.get_axis(3))
    motors["lift_motor"].setVelocity(right_y)

    # LT (open gripper)
    lt = joystick.get_button(1)  # A button to open gripper
    rt = joystick.get_button(0)  # B button to close gripper

    if lt:  # If A button pressed (open gripper)
        print("Opening gripper")
        gripper["grip_motor1"].setPosition(GRIPPER_OPEN_POSITION)  # Open gripper
        gripper["grip_motor2"].setPosition(GRIPPER_OPEN_POSITION)
    elif rt:  # If B button pressed (close gripper)
        print("Closing gripper")
        gripper["grip_motor1"].setPosition(GRIPPER_CLOSE_POSITION)  # Close gripper
        gripper["grip_motor2"].setPosition(GRIPPER_CLOSE_POSITION)

    # RB (rotate turret)
    rb_pressed = joystick.get_button(5)  # RB is button 5
    lb_pressed = joystick.get_button(4)  # LB is button 4
    if rb_pressed:
        motors["turret_motor"].setVelocity(1.0)  # Rotate turret motor
    elif lb_pressed:
        motors["turret_motor"].setVelocity(-1.0)  # Rotate turret motor
    else:
        motors["turret_motor"].setVelocity(0)  # Stop turret rotation

# Main loop
def main():
    joystick = initialize_gamepad()

    while robot.step(CONTROL_STEP) != -1:
        handle_gamepad_input(joystick)

if __name__ == "__main__":
    main()
