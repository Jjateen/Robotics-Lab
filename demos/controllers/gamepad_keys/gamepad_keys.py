import pygame
import sys
import os

# Mapping of button indices to their names
BUTTON_MAPPING = {
    0: "A",
    1: "B",
    2: "X",
    3: "Y",
    4: "LB",
    5: "RB",
    6: "Back",
    7: "Start",
    8: "L3",
    9: "R3",
    10: "HOME"
}

# Mapping of axis indices to their descriptions
AXIS_MAPPING = {
    0: "Left Joystick X (Left/Right)",
    1: "Left Joystick Y (Up/Down)",
    2: "Right Joystick X (Left/Right)",
    3: "Right Joystick Y (Up/Down)",
    4: "LT (Left Trigger)",
    5: "RT (Right Trigger)"
}

# Mapping of hat indices to their names
HAT_MAPPING = {
    0: "D-Pad"
}

def clear_console():
    """Clear the console for better readability."""
    os.system('cls' if os.name == 'nt' else 'clear')

def initialize_gamepad():
    """Initialize the pygame library and the gamepad."""
    pygame.init()
    pygame.joystick.init()

    # Check for connected joysticks
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joystick detected. Please connect your gamepad and try again.")
        pygame.quit()
        sys.exit()

    # List all connected joysticks
    print(f"Number of joysticks detected: {joystick_count}")
    for i in range(joystick_count):
        joystick = pygame.joystick.Joystick(i)
        joystick.init()
        print(f"{i}: {joystick.get_name()}")

    # Use the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"\nUsing joystick: {joystick.get_name()}")
    return joystick

def get_hat_directions(hat_position):
    """Convert hat position tuple to a list of direction strings."""
    directions = []
    x, y = hat_position
    if y == 1:
        directions.append("Up")
    elif y == -1:
        directions.append("Down")
    if x == -1:
        directions.append("Left")
    elif x == 1:
        directions.append("Right")
    return directions

def display_gamepad_data(joystick):
    """Continuously read and display joystick axes, button states, and D-Pad directions."""
    try:
        while True:
            # Event processing
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    raise KeyboardInterrupt

            # Get number of axes, buttons, and hats
            num_axes = joystick.get_numaxes()
            num_buttons = joystick.get_numbuttons()
            num_hats = joystick.get_numhats()

            # Read axis values
            axes = [joystick.get_axis(i) for i in range(num_axes)]

            # Read button states
            buttons = [joystick.get_button(i) for i in range(num_buttons)]

            # Read hat states
            hats = [joystick.get_hat(i) for i in range(num_hats)]

            # Clear the console for better readability
            clear_console()

            # Display Axes
            print("=== Axes ===")
            for i in range(6):  # Based on the provided mapping, we consider axes 0-5
                if i in AXIS_MAPPING:
                    axis_name = AXIS_MAPPING[i]
                else:
                    axis_name = f"Axis {i}"
                if i < len(axes):
                    axis_value = axes[i]
                    # For triggers (LT and RT), you might want to scale or adjust the display
                    if i in [4, 5]:
                        # Triggers typically range from -1 to 1 or 0 to 1 depending on the controller
                        # Adjusting to 0 to 1 for better readability
                        trigger_value = (axis_value + 1) / 2  # Scale from [-1,1] to [0,1]
                        print(f"{axis_name}: {trigger_value:.3f}")
                    else:
                        print(f"{axis_name}: {axis_value:.3f}")
                else:
                    print(f"{axis_name}: Not Available")

            # Display D-Pad (Hat) States
            print("\n=== D-Pad (Hat) ===")
            if num_hats == 0:
                print("No D-Pad detected.")
            else:
                for i in range(num_hats):
                    if i in HAT_MAPPING:
                        hat_name = HAT_MAPPING[i]
                    else:
                        hat_name = f"Hat {i}"
                    hat_position = hats[i]
                    directions = get_hat_directions(hat_position)
                    if directions:
                        directions_str = ", ".join(directions)
                    else:
                        directions_str = "Center"
                    print(f"{hat_name}: {directions_str}")

            # Display Buttons
            print("\n=== Buttons ===")
            for i in range(11):  # Based on the provided mapping, buttons 0-10
                if i in BUTTON_MAPPING:
                    button_name = BUTTON_MAPPING[i]
                else:
                    button_name = f"Button {i}"
                if i < len(buttons):
                    button_state = "Pressed" if buttons[i] else "Released"
                    print(f"{button_name}: {button_state}")
                else:
                    print(f"{button_name}: Not Available")

            # Add a small delay to prevent flooding the console
            pygame.time.wait(100)  # Wait for 100 milliseconds

    except KeyboardInterrupt:
        print("\nExiting...")
    finally:
        # Clean up
        joystick.quit()
        pygame.joystick.quit()
        pygame.quit()

if __name__ == "__main__":
    joystick = initialize_gamepad()
    display_gamepad_data(joystick)
