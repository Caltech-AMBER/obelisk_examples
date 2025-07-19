from enum import IntEnum

# Button mapping for Xbox Series X Controller
class Button(IntEnum):
    A = 0
    B = 1
    X = 2
    Y = 3
    LEFT_BUMPER = 4
    RIGHT_BUMPER = 5
    BACK = 6
    START = 7
    GUIDE = 8
    SHARE = 11

# Axis mapping for Xbox Series X Controller
class Axis(IntEnum):
    # Left stick buttons
    LEFT_X = 0 # Positive if stick is pushed leftward
    LEFT_Y = 1 # Positive if stick is pushed upward

    # Right stick buttons
    RIGHT_X = 3 # Positive if stick is pushed leftward
    RIGHT_Y = 4 # Positive if stick is pushed upward

    # Triggers
    LEFT_TRIGGER = 2 # Decreases from 1 to -1 as trigger is pressed
    RIGHT_TRIGGER = 5 # Decreases from 1 to -1 as trigger is pressed

    # Directional Pad
    DPAD_X = 6 # Positive if stick is pushed leftward
    DPAD_Y = 7 # Positive if stick is pushed upward