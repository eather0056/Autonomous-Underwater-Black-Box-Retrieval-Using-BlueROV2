#!/usr/bin/env python3

import pygame
from pygame.locals import *
import time

def joystick_tester():
    # Initialize pygame and the joystick module
    pygame.init()
    pygame.joystick.init()

    # Check if any joysticks are connected
    joystick_count = pygame.joystick.get_count()
    if joystick_count == 0:
        print("No joysticks connected. Please connect a joystick and try again.")
        return

    # Initialize the first joystick
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    print(f"Joystick detected: {joystick.get_name()}")

    # Display the number of axes, buttons, and hats
    print(f"Number of axes: {joystick.get_numaxes()}")
    print(f"Number of buttons: {joystick.get_numbuttons()}")
    print(f"Number of hats: {joystick.get_numhats()}")

    print("\nPress buttons, move axes, or use hats to see their values. Press Ctrl+C to exit.")

    try:
        while True:
            # Process events
            for event in pygame.event.get():
                if event.type == JOYAXISMOTION:
                    print(f"Axis {event.axis} moved to {event.value:.2f}")
                elif event.type == JOYBUTTONDOWN:
                    print(f"Button {event.button} pressed.")
                elif event.type == JOYBUTTONUP:
                    print(f"Button {event.button} released.")
                elif event.type == JOYHATMOTION:
                    print(f"Hat {event.hat} moved to {event.value}")

            time.sleep(0.1)  # Avoid spamming the output
    except KeyboardInterrupt:
        print("\nExiting joystick tester.")
    finally:
        pygame.quit()


if __name__ == "__main__":
    joystick_tester()
