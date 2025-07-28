from enum import Enum

class Mode(Enum):
    INIT = "init"
    MOVING_BY_JOYSTICK = "moving by joystick"
    MOVING_BY_GOAL_COMMAND = "moving by goal command"
    WAITING = "waiting"
    