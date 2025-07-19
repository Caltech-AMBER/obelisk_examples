import numpy as np
from d1_control.utils.constants import *
from d1_control.utils.ControlUtils import limit

class Joystick():
    def __init__(self):
        self.vcmd = np.zeros(3) # initial velocity command from joystick
        self.wcmd = np.zeros(3) # initial angular velocity command from joystick
        self._vgripper = 0 # initial gripper velocity
        self._speed = 0 # initial joystick speed
    
    @property
    def speed(self) -> float:
        return self._speed
    
    @speed.setter
    def speed(self, value: float):
        """Set the joy speed and ensure it doesn't exceed `MAX_JOY_SPEED`."""
        self._speed = limit(value, 0, MAX_JOY_SPEED)

    @property
    def vx(self) -> float:
        return self.vcmd[0]
    
    @vx.setter
    def vx(self, value: float):
        self.vcmd[0] = value

    @property
    def vy(self) -> float:
        return self.vcmd[1]
    
    @vy.setter
    def vy(self, value: float):
        self.vcmd[1] = value

    @property
    def vz(self) -> float:
        return self.vcmd[2]
    
    @vz.setter
    def vz(self, value: float):
        self.vcmd[2] = value

    def get_vcmd(self) -> np.ndarray:
        return self.vcmd
    
    @property
    def vgripper(self) -> float:
        return self._vgripper
    
    @vgripper.setter
    def vgripper(self, value):
        self._vgripper = value

    @property
    def wx(self) -> float:
        return self.wcmd[0]
    
    @wx.setter
    def wx(self, value: float):
        self.wcmd[0] = value

    @property
    def wy(self) -> float:
        return self.wcmd[1]
    
    @wy.setter
    def wy(self, value: float):
        self.wcmd[1] = value

    @property
    def wz(self) -> float:
        return self.wcmd[2]
    
    @wz.setter
    def wz(self, value: float):
        self.wcmd[2] = value

    def get_wcmd(self) -> np.ndarray:
        return self.wcmd
    
    