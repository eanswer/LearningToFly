import numpy as np
import math

class Wing:
    def __init__(self, area, direction, angle0):
        self.area = area
        self.direction = direction
        self.angle0 = angle0
        self.air_rho = 1.226

    def compute_aerodynamics(self, orientation, velocity, noisy = False):
        
        downward_direction = np.array([-self.direction[2], self.direction[1], self.direction[0]])

        R = orientation.rotation_matrix

        velocity_body = R.transpose().dot(velocity)
        velocity_body_projected = np.array([velocity_body[0], 0, velocity_body[2]])

        if np.linalg.norm(velocity_body_projected) < 0.01:
            lift = np.zeros(3)
            drag = np.zeros(3)
            return lift, drag, 0.0
        
        lift_dir_body = np.array([velocity_body[2], 0.0, -velocity_body[0]])
        drag_dir_body = -velocity_body
        lift_dir_body /= np.linalg.norm(lift_dir_body)
        drag_dir_body /= np.linalg.norm(drag_dir_body)

        lift_dir = R.dot(lift_dir_body)
        drag_dir = R.dot(drag_dir_body)

        angle_of_attack = math.asin(velocity_body_projected.dot(downward_direction) / np.linalg.norm(velocity_body_projected)) + self.angle0

        lift = self.get_lift(angle_of_attack, np.linalg.norm(velocity_body_projected), noisy = noisy) * lift_dir
        drag = self.get_drag(angle_of_attack, np.linalg.norm(velocity_body_projected), noisy = noisy) * drag_dir

        return lift, drag, angle_of_attack
    
    def get_lift(self, aoa, v, noisy = False):
        Cl = 2.0 * math.sin(aoa) * math.cos(aoa)
        lift_force = Cl * self.air_rho * (v ** 2) * self.area / 2.0
        if noisy == False:
            return lift_force
        else:
            return lift_force * np.random.uniform(low = 0.8, high = 1.2)
    
    def get_drag(self, aoa, v, noisy = False):
        Cd = 2.0 * (math.sin(aoa) ** 2)
        if noisy:
            Cd = 2.0 * (math.sin(aoa) ** 2) + np.random.uniform(low = 0.0, high = 0.2)
        drag_force = Cd * self.air_rho * (v ** 2) * self.area / 2.0
        if noisy == False:
            return drag_force
        else:
            return drag_force * np.random.uniform(low = 0.9, high = 1.2)

