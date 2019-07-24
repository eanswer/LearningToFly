from pyquaternion import Quaternion
import numpy as np
import math

def angle2radians(angle):
    return angle / 180.0 * math.pi

def radians2angle(radians):
    return radians / math.pi * 180.0

def rpy_to_rotation_matrix(rpy):
    roll = rpy[0]
    pitch = rpy[1] 
    yaw = rpy[2]

    R_roll = np.array([[1, 0, 0], [0, math.cos(roll), -math.sin(roll)], [0, math.sin(roll), math.cos(roll)]])

    R_pitch = np.array([[math.cos(pitch), 0, math.sin(pitch)], [0, 1, 0], [-math.sin(pitch), 0, math.cos(pitch)]])

    R_yaw = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])

    return R_yaw.dot(R_pitch).dot(R_roll)

def rpy_to_angle_axis(rpy):
    quat = rpy_to_quaternion(rpy)
    len = quat[0] ** 2 + quat[1] ** 2 + quat[2] ** 2 + quat[3] ** 2
    quat /= len
    print("quat = ", quat)

    angle = 2.0 * math.acos(quat[0])
    while (angle < -math.pi):
        angle += math.pi * 2.0
    while (angle > math.pi):
        angle -= math.pi * 2.0

    a = math.sqrt(1.0 - quat[0] * quat[0])
    axis = np.array([quat[1] / a, quat[2] / a, quat[3] / a])
    print("angle = ", angle)
    print("axis = ", axis)

    return axis * angle

def quaternion_to_rpy(attitude_quaternion):
    # based on wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    attitude_quaternion._normalise()

    w = attitude_quaternion[0]
    x = attitude_quaternion[1]
    y = attitude_quaternion[2]
    z = attitude_quaternion[3]

    t0 = 2.0 * (w * x + y * z)
    t1 = 1.0 - 2.0 * (x * x + y * y)
    X = math.atan2(t0, t1)
    
    t2 = 2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    Y = math.asin(t2)

    t3 = 2.0 * (w * z + x * y)
    t4 = 1.0 - 2.0 * (y * y + z * z)
    Z = math.atan2(t3, t4)
    
    return np.array([X, Y, Z])

def rpy_to_quaternion(rpy):
    # based on wikipedia: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
    roll = rpy[0]
    pitch = rpy[1]
    yaw = rpy[2]

    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)

    w = cy * cr * cp + sy * sr * sp
    x = cy * sr * cp - sy * cr * sp
    y = cy * cr * sp + sy * sr * cp
    z = sy * cr * cp - cy * sr * sp

    quat = Quaternion(w, x, y, z)
    quat._normalise()
    
    return quat

def angular_rate_to_euler_rate_matrix(rpy):
    roll = rpy[0]
    pitch = rpy[1]
    s_roll = math.sin(roll)
    c_roll = math.cos(roll)
    c_pitch = math.cos(pitch)
    t_pitch = math.tan(pitch)
    return np.array([[1, s_roll * t_pitch, c_roll * t_pitch], [0, c_roll, -s_roll], [0, s_roll / c_pitch, c_roll / c_pitch]])

def compute_rpy_rate(rpy, angular_rate):
    # problematic, the incoming angular rate is not in body frame, but it will not affect the simulation
    return angular_rate_to_euler_rate_matrix(rpy).dot(angular_rate)

# testing
import random

def test_rpy_quaternion():
    # testing rpy-quaternion conversion
    rpy = np.zeros(3)
    rpy[0] = random.uniform(-math.pi, math.pi )
    rpy[1] = random.uniform(-math.pi / 2.0, math.pi / 2.0)
    rpy[2] = random.uniform(-math.pi, math.pi)

    orientation = rpy_to_quaternion(rpy)
    inverse_rpy = quaternion_to_rpy(orientation)

    roll_quat = Quaternion(axis = [1, 0, 0], angle = rpy[0])
    pitch_quat = Quaternion(axis = [0, 1, 0], angle = rpy[1])
    yaw_quat = Quaternion(axis = [0, 0, 1], angle = rpy[2])
    
    orientation_from_rpy = yaw_quat * pitch_quat * roll_quat

    print("-------------test rpy to orientation-----------------")
    print("rpy = ", rpy, ", inverse rpy = ", inverse_rpy, ", norm(diff) = ", np.linalg.norm(rpy - inverse_rpy))
    print("orientation = ", orientation, ", another orientation = ", orientation_from_rpy, ", diff = ", orientation - orientation_from_rpy)

def test_rpy_rotation_matrix():
    # testing rpy-rotation conversion
    rpy = np.zeros(3)
    rpy[0] = random.uniform(-math.pi, math.pi )
    rpy[1] = random.uniform(-math.pi / 2.0, math.pi / 2.0)
    rpy[2] = random.uniform(-math.pi, math.pi)

    orientation = rpy_to_quaternion(rpy)

    rotation_from_quat = orientation.rotation_matrix

    rotation = rpy_to_rotation_matrix(rpy)
    
    print("-------------test rpy to rotation-----------------")
    print("rpy = ", rpy)
    print("rotation = \n", rotation)
    print("rotation from quat = \n", rotation_from_quat)
    print("diff = \n", rotation - rotation_from_quat)
    if (np.linalg.norm(rotation - rotation_from_quat) > 1e-7):
        input()

def test_rpy_angle_axis():
    # testing rpy-angle axis conversion
    rpy = np.zeros(3)
    rpy[0] = random.uniform(-math.pi / 2.0, math.pi / 2.0)
    rpy[1] = random.uniform(-math.pi / 2.0, math.pi / 2.0)
    rpy[2] = random.uniform(0, math.pi * 2.0)
    # rpy[0] = 3.0664
    # rpy[1] = 0.17985
    # rpy[2] = -2.4785

    orientation = rpy_to_quaternion(rpy)
    print("quat = ", orientation)
    print("angle = ", orientation.angle)
    print("axis = ", orientation.axis)
    
    angleaxis_from_quat = orientation.angle * orientation.axis
    
    

    angleaxis = rpy_to_angle_axis(rpy)

    print("-------------test rpy to angle axis-----------------")
    print("rpy = ", rpy)
    print("angle axis = \n", angleaxis)
    print("angle axis from quat = \n", angleaxis_from_quat)
    print("diff = \n", angleaxis - angleaxis_from_quat)
    if (np.linalg.norm(angleaxis - angleaxis_from_quat) > 1e-7):
        input()

def main():
    for i in range(1000):
        # test_rpy_quaternion()
        
        # test_rpy_rotation_matrix()

        test_rpy_angle_axis()

if __name__=='__main__':
    main()