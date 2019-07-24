import numpy as np

class Rotor:
    def __init__(self, position_body, direction_body, clockwise, torque_coef):
        self.position = position_body
        self.direction = direction_body
        self.direction = self.direction / np.linalg.norm(self.direction)
        self.clockwise = clockwise
        self.torque_coef = torque_coef
