import mujoco_rendering_env
from mujoco_rendering_env import mujoco_env
import rotor
from rotor import Rotor
import wing
from wing import Wing
from Utils import rigid_body
from Utils import euler_angle

import xml.etree.ElementTree as ET
import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
from pyquaternion import Quaternion
import matplotlib.pyplot as plt
import seaborn as sns

def warp_PI(radians):
    res = np.zeros(len(radians))
    for i in range(len(radians)):
        res[i] = radians[i]
        while res[i] < -math.pi:
            res[i] += 2.0 * math.pi
        while res[i] > math.pi:
            res[i] -= 2.0 * math.pi
    return res

class Hybrid3DEnv(gym.Env):
    def __init__(self, data_folder, config_file, play):
        self.render_filename = None
        self.mass = None
        self.inertia_tensor = None
        self.rotors = []
        self.wing = None

        # initialize constant value
        self.gravity = np.array([0, 0, 9.8])
        self.dt_mean = 0.01
        self.dt_std = 0.005
        self.total_iter = 2000

        # parse xml config file
        self.parse_config_file(data_folder + config_file)
        
        # construct rendering environment
        self.render_env = mujoco_env.MujocoEnv(model_path = data_folder + self.render_filename)
        self.render_intervel = int(1.0 / 50.0 / self.dt_mean)

        # self.render_env._get_viewer()
        
        # noise related
        self.noisy_body = True
        self.noisy_sensor = True
        self.state_noise_std = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.005, 0.005, 0.0005, 0.0005, 0.0005])
        self.noisy_rotor = False
        self.noisy_aerodynamics = True
        self.simulate_delay = True
        self.delay = 0.04
        self.delay_mean = 0.04
        self.delay_std = 0.01
        self.noisy_dt = True
        self.constrain_motor_output = True
        self.motor_constrain_clip = 0.4
        self.real_rotors = []
        for i in range(len(self.rotors)):
            self.real_rotors.append(Rotor(self.rotors[i].position, self.rotors[i].direction, self.rotors[i].clockwise, self.rotors[i].torque_coef))

        # integral term
        self.I_dt = self.dt_mean
        self.I_error = None

        # mass property
        self.real_mass = self.mass
        self.real_inertia_tensor = self.inertia_tensor.copy()

        # initialize rigid body
        self.rigid_body = rigid_body.RigidBody(mass = self.mass, inertia_body = self.inertia_tensor)
        self.state = None

        # train or play
        self.play = play

        # training variables
        self.seed()
        self.iter = None
        self.epoch = 0
        self.timesofar = None
        self.target = np.zeros(4)
        self.state_his = None
        self.I_error = np.zeros(4)

        # construct action space
        self.max_thrust = 7.0
        action_low = np.ones(len(self.rotors)) * -1.0 * self.max_thrust / 2.0
        action_high = np.ones(len(self.rotors)) * self.max_thrust / 2.0
        self.action_space = spaces.Box(action_low, action_high, dtype=np.float32)

        # construct observation space
        ob = self.get_observation_vector()
        ob_low = np.ones(len(ob)) * (-np.finfo(np.float32).max)
        ob_high = np.ones(len(ob)) * (np.finfo(np.float32).max)
        self.observation_space = spaces.Box(ob_low, ob_high, dtype=np.float32)
    
    #########################################################################
    # parse config file
    def parse_config_file(self, config_file):
        xml_tree = ET.parse(config_file)
        root = xml_tree.getroot()
        self.parse_xml_tree(root)

    def parse_xml_tree(self, root):
        # parse node
        if root.tag == "render_file":
            self.render_filename = root.attrib["filename"]

        if root.tag == "mass_property":
            self.mass = float(root.attrib["mass"])
            self.inertia_tensor = self.convert_str_to_matrix(root.attrib["inertia_tensor"], 3, 3)
        
        if root.tag == "rotor":
            pos = self.convert_str_to_vector(root.attrib["position"], 3)
            dir = self.convert_str_to_vector(root.attrib["direction"], 3)
            clockwise = (root.attrib["clockwise"] == "1")
            torque_coef = float(root.attrib["torque_coef"])
            rotor = Rotor(position_body = pos, direction_body = dir, clockwise = clockwise, torque_coef = torque_coef)
            self.rotors.append(rotor)

        if root.tag == "wing":
            area = float(root.attrib["area"])
            dir = self.convert_str_to_vector(root.attrib["direction"], 3)
            angle0 = math.radians(float(root.attrib["angle0"]))
            wing = Wing(area = area, direction = dir, angle0 = angle0)
            self.wing = wing

        # search sub-tree
        for child in root:
            self.parse_xml_tree(child)
    
    def convert_str_to_matrix(self, string, dim0, dim1):
        a = string.split(' ')
        assert len(a) == dim0 * dim1
        res = np.zeros([dim0, dim1])
        for i in range(dim0):
            for j in range(dim1):
                res[i, j] = float(a[i * dim1 + j])
        return res

    def convert_str_to_vector(self, string, dim):
        a = string.split(' ')
        assert len(a) == dim
        res = np.zeros(dim)
        for i in range(len(a)):
            res[i] = float(a[i])
        return res
    #########################################################################

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]
    
    def wrap2PI(self, x):
        while x > math.pi:
            x -= math.pi * 2.0
        while x < -math.pi:
            x += math.pi * 2.0
        return x

    def calc_local_velocity(self, yaw, vel):
        vel_local = np.zeros(3)
        vel_local[0] = vel[0] * math.cos(yaw) + vel[1] * math.sin(yaw)
        vel_local[1] = -vel[0] * math.sin(yaw) + vel[1] * math.cos(yaw)
        vel_local[2] = vel[2]
        return vel_local

    def get_observation_vector(self):
        if self.state_his is None:
            # For observation space construction
            state = np.zeros(12)
        else:
            if self.simulate_delay:
                if self.timesofar > self.delay:
                    for i in reversed(range(len(self.state_his))):
                        if self.time_his[i] <= self.timesofar - self.delay:
                            state = self.state_his[i]
                            break
                else:
                    state = self.state_his[0]
            else:
                state = self.state_his[len(self.state_his) - 1]

        rpy = state[3:6]
        vel = state[6:9]
        omega = state[9:12]

        now = np.hstack((vel, rpy[2]))
        error = self.target - now

        coeff = 0.999
        self.I_error = coeff * self.I_error + self.I_dt * error
        self.I_error = np.clip(self.I_error, -10.0, 10.0)

        ob = np.hstack((rpy, omega, error[0:3], self.I_error))

        if (ob[0] < -math.pi or ob[1] < -math.pi or ob[2] < -math.pi or ob[0] > math.pi or ob[1] > math.pi or ob[2] > math.pi):
            print("ob wrong: ", ob)
            input()

        return ob

    def update_state(self):
        # get local vel
        vel_local = self.calc_local_velocity(self.rigid_body.rpy[2], self.rigid_body.velocity)
        
        now_state = np.concatenate([self.rigid_body.position, self.rigid_body.rpy, vel_local, self.rigid_body.omega_body])

        # add noise
        if self.noisy_sensor:
            noise = np.random.normal(self.state_noise_mean, self.state_noise_std)
            now_state = now_state + noise

        now_state[3] = self.wrap2PI(now_state[3])
        now_state[4] = self.wrap2PI(now_state[4])
        now_state[5] = self.wrap2PI(now_state[5])
        
        self.state_his.append(now_state)
        self.time_his.append(self.timesofar)
    
    def reset_noise(self):
        if self.noisy_body:
            self.mass = np.random.uniform(low = 0.95, high = 1.05) * self.real_mass
            for i in range(3):
                for j in range(3):
                    if i <= j:
                        self.inertia_tensor[i][j] = self.inertia_tensor[j][i] = self.real_inertia_tensor[i][j] * np.random.uniform(low = 0.6, high = 1.4)

        state_noise_mean_range = np.array([0.0, 0.0, 0.0, 0.02, 0.02, 0.1, 0.2, 0.2, 0.2, 0.00, 0.00, 0.00])
        self.state_noise_mean = np.random.uniform(low = -1.0 * state_noise_mean_range, high = state_noise_mean_range, size = 12)

    def reset(self):
        # generate epoch-level noise
        self.reset_noise()

        # set initial state
        initial_pos = np.array([0, 0, -4])

        initial_orientation = Quaternion(axis = [1, 0, 0], angle = 0)

        axis = np.random.uniform(low = -1.0, high = 1.0, size = 3)
        angle = np.random.normal(0.0, 0.2)
        initial_orientation = Quaternion(axis = axis, angle = angle)

        initial_velocity = np.zeros(3)

        initial_velocity = np.random.normal(0.0, 0.5, size = 3)

        initial_omega = np.zeros(3)

        initial_omega = np.random.normal(0.0, 0.05, size = 3)

        self.rigid_body = rigid_body.RigidBody(mass = self.mass, inertia_body = self.inertia_tensor, \
                                                position = initial_pos, orientation = initial_orientation, \
                                                velocity = initial_velocity, angular_velocity = initial_omega)
        
        mode = self.np_random.uniform(low = 0.0, high = 2.0)
        if mode < 1.0:    
            self.target_vx = self.np_random.uniform(low = 3.0, high = 6.0)
            self.target_vy = 0.0
            self.target_vz = self.np_random.uniform(low = -1.0, high = 1.0)
        else:
            self.target_vx = self.np_random.uniform(low = -1.0, high = 1.0)
            self.target_vy = self.np_random.uniform(low = -1.0, high = 1.0)
            self.target_vz = self.np_random.uniform(low = -1.0, high = 1.0)
        self.target = np.array([self.target_vx, self.target_vy, self.target_vz, 0])

        # training variables
        self.epoch += 1
        self.iter = 0
        self.timesofar = 0
        self.last_action = np.zeros(len(self.rotors))
        self.I_error = np.zeros(4)
        self.I_error = np.random.normal(0.0, 0.5, size = 4)
        
        # for visualization
        self.accumulate_reward = 0
        self.rpy_his = [[], [], []]
        self.vel_local_his = [[], [], []]
        self.target_vel_his = [[], [], []]
        self.action_his = [[], [], [], []]
        self.omega_his = [[], [], []]
        self.state_his = []
        self.time_his = []
        self.error_his = [[], [], [], []]
        self.I_error_his = [[], [], [], []]
        self.aoa_his = []
        self.reward_his = None

        self.update_state()
        observation_vector = self.get_observation_vector()

        return observation_vector

    def compute_reward(self):
        alive_bonus = 400.0
        
        vel_local = self.calc_local_velocity(self.rigid_body.rpy[2], self.rigid_body.velocity)

        vel_cost = 50.0 * np.square((vel_local - self.target[0:3])).sum()
        
        orientation_cost = 150.0 * (self.wrap2PI(self.rigid_body.rpy[2]) ** 2)

        control_cost = 0.2 * np.square(self.last_action).sum()

        omega_cost = 5.0 * np.square(self.rigid_body.omega).sum()

        I_cost = 6.0 * np.square(self.I_error).sum()
        # I_cost = 0.0

        reward = (alive_bonus - control_cost - vel_cost - orientation_cost - omega_cost - I_cost) * self.dt

        if self.reward_his is None:
            self.reward_his = np.array([control_cost, vel_cost, orientation_cost, omega_cost, I_cost]) * self.dt
        else:
            self.reward_his += np.array([control_cost, vel_cost, orientation_cost, omega_cost, I_cost]) * self.dt

        return reward

    def step(self, action):
        assert len(action) == len(self.rotors)

        # add noise to dt
        if self.noisy_dt:
            self.dt = np.random.normal(self.dt_mean, self.dt_std)
        else:
            self.dt = self.dt_mean

        if self.simulate_delay:
            self.delay = np.random.normal(self.delay_mean, self.delay_std)

        # apply gravity
        self.rigid_body.apply_force_in_world_frame(force = self.mass * self.gravity, position = self.rigid_body.point_body_to_world(body_point = np.zeros(3)))
        
        # apply rotors forces / torques
        for i in range(len(self.rotors)):
            action[i] += self.max_thrust / 2.0
            
            if self.noisy_rotor:
                action[i] *= np.random.uniform(low = 0.8, high = 1.2)

            action[i] = np.clip(action[i], 0.0, self.max_thrust)

            # constrain motor output
            if self.constrain_motor_output:
                action[i] = np.clip(action[i], self.last_action[i] - self.motor_constrain_clip, self.last_action[i] + self.motor_constrain_clip)

            self.rigid_body.apply_force_in_body_frame(force = action[i] * self.rotors[i].direction, \
                                                        position = self.rotors[i].position)
            
            if self.rotors[i].clockwise:
                self.rigid_body.apply_torque(torque = self.rotors[i].torque_coef * action[i] * self.rigid_body.vector_body_to_world(self.rotors[i].direction))
            else:
                self.rigid_body.apply_torque(torque = -1.0 * self.rotors[i].torque_coef * action[i] * self.rigid_body.vector_body_to_world(self.rotors[i].direction))

        # apply aerodynamics
        lift, drag, aoa = self.wing.compute_aerodynamics(orientation = self.rigid_body.orientation, velocity = self.rigid_body.velocity, noisy = self.noisy_aerodynamics)
        AC = np.zeros(3)
        self.rigid_body.apply_force_in_world_frame(force = lift, position = self.rigid_body.point_body_to_world(body_point = AC))
        self.rigid_body.apply_force_in_world_frame(force = drag, position = self.rigid_body.point_body_to_world(body_point = AC))
        
        # advance simulation
        self.last_action = action
        self.rigid_body.advance(self.dt)
        self.timesofar += self.dt
        self.iter += 1

        # update state and get observation vector
        self.update_state()
        ob = self.get_observation_vector()

        # check if fails (e.g. copter flipped over)
        done = False
        done = done or (np.linalg.norm(self.rigid_body.velocity) > 10)
        done = done or (np.linalg.norm(self.rigid_body.omega) > 4.0 * math.pi)
        done = done or (abs(self.rigid_body.velocity_body[1]) > 2)

        done = done or (self.iter >= self.total_iter)

        # compute reward
        reward = self.compute_reward()

        # render in render_env
        if self.play and self.iter % self.render_intervel == 0:
            # reset camera
            if not (self.render_env.viewer is None):
                self.render_env.viewer.cam.lookat[1] = self.rigid_body.position[0]
                self.render_env.viewer.cam.lookat[0] = self.rigid_body.position[1]
                self.render_env.viewer.cam.lookat[2] = -self.rigid_body.position[2]

            # update object
            qpos = np.zeros(7)
            qpos[0] = self.rigid_body.position[1]
            qpos[1] = self.rigid_body.position[0]
            qpos[2] = - self.rigid_body.position[2]

            # compute transformation in visualization coordinates
            NED_orientation = self.rigid_body.orientation
            NED_R = NED_orientation.rotation_matrix
            NED2VIS_R = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])

            VIS_R = np.matmul(NED2VIS_R, np.matmul(NED_R, np.linalg.inv(NED2VIS_R)))
            VIS_Orientation = Quaternion(matrix = VIS_R)

            qpos[3] = VIS_Orientation[0]
            qpos[4] = VIS_Orientation[1]
            qpos[5] = VIS_Orientation[2]
            qpos[6] = VIS_Orientation[3]
            
            self.render_env.set_state(qpos)
            self.render_env.render()

        return ob, np.sum(reward), done, {}

    def close(self):
        if self.render_env:
            self.render_env.close()
            self.render_env = None

        
    
