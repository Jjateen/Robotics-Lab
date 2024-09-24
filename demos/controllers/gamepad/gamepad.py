import pygame
import sys
from controller import Robot, Motor

# Constants
TIME_STEP = 16
NUM_MOTORS = 12
NUM_STATES = 6

# Speed and position limits
FRONT = +0.7  # For hip motors
BACK = -0.7   # For hip motors
HI = +0.02    # For knee motors
LO = -0.02    # For knee motors

# Joystick dead zone threshold
DEAD_ZONE = 0.1  # Adjust this value based on your joystick's sensitivity

# Joystick axes for movement
AXIS_LEFT_X = 0  # Left joystick X axis (left/right for turning)
AXIS_LEFT_Y = 1  # Left joystick Y axis (up/down for forward/backward)

# Motor names in the hexapod
MOTOR_NAMES = ["hip_motor_l0", "hip_motor_l1", "hip_motor_l2", "hip_motor_r0",
               "hip_motor_r1", "hip_motor_r2", "knee_motor_l0", "knee_motor_l1",
               "knee_motor_l2", "knee_motor_r0", "knee_motor_r1", "knee_motor_r2"]

# Predefined leg positions for the tripod gait (forward and backward sequences)
# Hip motor positions for forward and backward movement
pos_hip_forward = [
    [BACK, FRONT, BACK, -FRONT, -BACK, -FRONT],  # State 0
    [BACK, FRONT, BACK, -FRONT, -BACK, -FRONT],  # State 1
    [BACK, FRONT, BACK, -FRONT, -BACK, -FRONT],  # State 2
    [FRONT, BACK, FRONT, -BACK, -FRONT, -BACK],  # State 3
    [FRONT, BACK, FRONT, -BACK, -FRONT, -BACK],  # State 4
    [FRONT, BACK, FRONT, -BACK, -FRONT, -BACK],  # State 5
]

# For reverse gait, the entire forward gait is reversed
pos_hip_backward = [
    [FRONT, BACK, FRONT, -BACK, -FRONT, -BACK],  # State 5
    [FRONT, BACK, FRONT, -BACK, -FRONT, -BACK],  # State 4
    [FRONT, BACK, FRONT, -BACK, -FRONT, -BACK],  # State 3
    [BACK, FRONT, BACK, -FRONT, -BACK, -FRONT],  # State 2
    [BACK, FRONT, BACK, -FRONT, -BACK, -FRONT],  # State 1
    [BACK, FRONT, BACK, -FRONT, -BACK, -FRONT],  # State 0
]

# Knee motor positions remain the same for forward and backward
pos_knee = [
    [LO, HI, LO, HI, LO, HI],  # State 0
    [HI, HI, HI, HI, HI, HI],  # State 1
    [HI, LO, HI, LO, HI, LO],  # State 2
    [HI, LO, HI, LO, HI, LO],  # State 3
    [HI, HI, HI, HI, HI, HI],  # State 4
    [LO, HI, LO, HI, LO, HI],  # State 5
]

# Custom lists for alternate tripod movement in turning
# Right turn hip motor positions
pos_hip_right_turn = [
    [FRONT, FRONT, BACK, -BACK, -FRONT, -FRONT],  # State 0
    [FRONT, FRONT, BACK, -BACK, -FRONT, -FRONT],  # State 1
    [BACK, BACK, FRONT, -FRONT, -BACK, -BACK],    # State 2
    [BACK, BACK, FRONT, -FRONT, -BACK, -BACK],    # State 3
    [FRONT, FRONT, BACK, -BACK, -FRONT, -FRONT],  # State 4
    [BACK, BACK, FRONT, -FRONT, -BACK, -BACK],    # State 5
]

# Left turn hip motor positions
pos_hip_left_turn = [
    [BACK, BACK, FRONT, -FRONT, -BACK, -BACK],    # State 0
    [BACK, BACK, FRONT, -FRONT, -BACK, -BACK],    # State 1
    [FRONT, FRONT, BACK, -BACK, -FRONT, -FRONT],  # State 2
    [FRONT, FRONT, BACK, -BACK, -FRONT, -FRONT],  # State 3
    [BACK, BACK, FRONT, -FRONT, -BACK, -BACK],    # State 4
    [FRONT, FRONT, BACK, -BACK, -FRONT, -FRONT],  # State 5
]

# Custom knee motor positions for turning (mirrored for left/right turns)
pos_knee_turn = [
    [HI, HI, HI, HI, HI, HI],  # State 0
    [LO, HI, LO, HI, LO, HI],  # State 1
    [HI, LO, HI, LO, HI, LO],  # State 2
    [HI, HI, HI, HI, HI, HI],  # State 3
    [LO, HI, LO, HI, LO, HI],  # State 4
    [HI, LO, HI, LO, HI, LO],  # State 5
]

def clamp(value, min_value, max_value):
    """Clamp value between min_value and max_value."""
    return max(min(value, max_value), min_value)

def initialize_gamepad():
    """Initialize the pygame library and the gamepad."""
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("No joystick detected.")
        pygame.quit()
        sys.exit()

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick

def apply_dead_zone(value, threshold=DEAD_ZONE):
    """Ignore joystick values that are within the dead zone around 0."""
    if abs(value) < threshold:
        return 0
    return value

def main():
    # Initialize Webots robot and motors
    robot = Robot()
    motors = [robot.getDevice(name) for name in MOTOR_NAMES]

    # Set motors to position control mode
    for motor in motors:
        motor.setPosition(float('inf'))  # Enable velocity control by setting infinite position
        motor.setVelocity(0.0)  # Default to stop

    # Initialize the gamepad
    joystick = initialize_gamepad()

    elapsed = 0  # To track the gait state transitions

    try:
        while robot.step(TIME_STEP) != -1:
            # Get joystick input
            pygame.event.pump()
            forward_back = joystick.get_axis(AXIS_LEFT_Y)  # Forward/backward on Y-axis
            left_right = joystick.get_axis(AXIS_LEFT_X)  # Left/right on X-axis

            # Apply dead zone to the joystick input
            forward_back = apply_dead_zone(forward_back)
            left_right = apply_dead_zone(left_right)

            # Invert forward_back for correct forward/backward motion
            forward_back = -forward_back

            # Debug: Print joystick values
            print(f"Joystick Y-axis value (with dead zone): {forward_back}")
            print(f"Joystick X-axis value (with dead zone): {left_right}")

            # Calculate gait state based on the time elapsed
            elapsed += 1
            state = (elapsed // 25) % NUM_STATES  # Cycle through the 6 gait states

            # Handle turning with higher priority than forward/backward
            if left_right != 0:
                # Alternate tripod turning logic
                speed = clamp(abs(left_right) * 3.0, 0, 3.0)
                if left_right > 0:
                    # Right turn using alternate tripod movement
                    for i in range(6):  # First 6 motors are hips
                        motors[i].setPosition(clamp(pos_hip_right_turn[state][i], BACK, FRONT))

                elif left_right < 0:
                    # Left turn using alternate tripod movement
                    for i in range(6):  # First 6 motors are hips
                        motors[i].setPosition(clamp(pos_hip_left_turn[state][i], BACK, FRONT))

                # Update knee motors for turning
                for i in range(6, 12):  # Last 6 motors are knees
                    motors[i].setPosition(clamp(pos_knee_turn[state][i - 6], LO, HI))

                # Apply speed to the motors for turning
                for i in range(NUM_MOTORS):
                    motors[i].setVelocity(speed)

            else:
                # Handle forward and backward movement when no turning is happening
                if forward_back < 0:
                    # Backward gait
                    for i in range(6):  # First 6 motors are hips
                        motors[i].setPosition(clamp(pos_hip_backward[state][i], BACK, FRONT))
                else:
                    # Forward gait
                    for i in range(6):  # First 6 motors are hips
                        motors[i].setPosition(clamp(pos_hip_forward[state][i], BACK, FRONT))

                # Update knee motors for forward/backward movement
                for i in range(6, 12):  # Last 6 motors are knees
                    motors[i].setPosition(clamp(pos_knee[state][i - 6], LO, HI))

                # Apply speed based on forward/backward input
                speed = clamp(abs(forward_back) * 3.0, 0, 3.0)
                for i in range(NUM_MOTORS):
                    motors[i].setVelocity(speed)

    except KeyboardInterrupt:
        # Graceful exit on keyboard interrupt
        print("Exiting program.")
        pygame.quit()
        sys.exit()


if __name__ == "__main__":
    main()