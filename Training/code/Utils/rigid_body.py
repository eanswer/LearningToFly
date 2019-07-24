import numpy as np
from pyquaternion import Quaternion
from Utils import euler_angle
import math

def quaternion_to_orientation_vector(quat):
    return quat.rotation_matrix.dot(np.array([0, 0, -1]))

def wrap2PI(radians):
    for i in range(len(radians)):
        while radians[i] < -math.pi:
            radians[i] += 2.0 * math.pi
        while radians[i] > math.pi:
            radians[i] -= 2.0 * math.pi
    return radians

class RigidBody:
    def __init__(self, mass, inertia_body, 
        position = np.zeros(3), velocity = np.zeros(3), 
        orientation = Quaternion(axis = [0, 0, 1], angle = 0), 
        angular_velocity = np.zeros(3)):

        self.dt = 0.05
        self.mass = mass
        self.inertia_body = inertia_body
        self.inertia_body_inv = np.linalg.inv(inertia_body)
        self.position = position
        self.velocity = velocity
        self.orientation = orientation

        self.linear_momentum = velocity * mass
        self.R = orientation.rotation_matrix # R is the rotation matrix from body frame to world frame
        self.omega = angular_velocity
        self.angular_momentum = self.R.dot(self.inertia_body).dot(self.R.transpose()).dot(self.omega)
        self.inertia_inv = self.R.dot(self.inertia_body_inv).dot(self.R.transpose())
        self.acceleration = np.zeros(3)
        self.external_force = np.zeros(3)
        self.external_torque = np.zeros(3)

        self.omega_body = self.R.transpose().dot(self.omega)
        self.velocity_body = self.R.transpose().dot(self.velocity)
        self.angle_axis = self.orientation.angle * self.orientation.axis
        self.rpy = euler_angle.quaternion_to_rpy(self.orientation)
        self.rpy = wrap2PI(self.rpy)
        self.rpy_rate = euler_angle.compute_rpy_rate(self.rpy, np.linalg.inv(self.R).dot(self.omega))

        self.orientation_vector = quaternion_to_orientation_vector(self.orientation)

    def wrap2PI(self, x):
        while x > math.pi:
            x -= math.pi * 2.0
        while x < -math.pi:
            x += math.pi * 2.0
        return x

    def apply_force_in_world_frame(self, force, position):
        self.external_force += force
        self.external_torque += np.cross(position - self.position, force)
    
    def apply_force_in_body_frame(self, force, position):
        self.apply_force_in_world_frame(self.vector_body_to_world(force), self.point_body_to_world(position))

    def apply_torque(self, torque):
        self.external_torque += torque
    
    def clear(self):
        self.external_force = np.zeros(3)
        self.external_torque = np.zeros(3)
    
    def advance_one_step(self, dt):
        self.position = self.position + self.velocity * dt
        
        omega = Quaternion(0.0, self.omega[0] * dt, self.omega[1] * dt, self.omega[2] * dt)
        orientation_dot = (omega * self.orientation) * 0.5
        self.orientation = self.orientation + orientation_dot
        self.orientation._normalise()

        self.linear_momentum += self.external_force * dt
        
        self.angular_momentum += self.external_torque * dt

        self.R = self.orientation.rotation_matrix
        self.inertia_inv = self.R.dot(self.inertia_body_inv).dot(self.R.transpose())
        self.velocity = self.linear_momentum / self.mass
        self.acceleration = self.external_force / self.mass
        self.omega = self.inertia_inv.dot(self.angular_momentum)

        self.omega_body = self.R.transpose().dot(self.omega)
        self.velocity_body = self.R.transpose().dot(self.velocity)
        self.angle_axis = self.orientation.angle * self.orientation.axis
        self.rpy = euler_angle.quaternion_to_rpy(self.orientation)
        self.rpy = wrap2PI(self.rpy)
        self.rpy_rate = euler_angle.compute_rpy_rate(self.rpy, np.linalg.inv(self.R).dot(self.omega))
        
        self.orientation_vector = quaternion_to_orientation_vector(self.orientation)

    def advance(self, dt):
        T = 0.0
        while T < dt:
            if self.dt < dt - T:
                one_step_dt = self.dt
            else:
                one_step_dt = dt - T
            self.advance_one_step(one_step_dt)
            T += one_step_dt
        self.clear()

    def vector_body_to_world(self, body_vector):
        return self.R.dot(body_vector)
    
    def vector_world_to_body(self, world_vector):
        return self.R.transpose().dot(world_vector)
        
    def point_body_to_world(self, body_point):
        return self.R.dot(body_point) + self.position