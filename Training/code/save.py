#!/usr/bin/env python3

import sys
sys.path.insert(0, '../../../../')

import tensorflow as tf
from mpi4py import MPI
from baselines.common import set_global_seeds
from baselines.common import tf_util as U
from baselines import logger
from baselines.common.cmd_util import make_robotics_env, robotics_arg_parser
import hybrid3D_env
from Utils import firmware_generator
import random
import time
import os
import numpy as np
import math
import argparse

def save(model_directory, model_filename):
    # check directory exist, if not create one
    if not os.path.isdir(model_directory):
        os.mkdir(model_directory)

    from baselines.ppo1 import mlp_policy, pposgd_simple
    U.make_session(num_cpu=1).__enter__()
    
    def policy_fn(name, ob_space, ac_space):
        return mlp_policy.MlpPolicy(name=name, ob_space=ob_space, ac_space=ac_space,
            hid_size=64, num_hid_layers=2)
    
    env = hybrid3D_env.Hybrid3DEnv(data_folder = "../../../data/", config_file = "E1.xml", play = True)

    pi = pposgd_simple.build_graph_only(env, policy_fn,
                max_timesteps=1000000,
                timesteps_per_actorbatch=4096,
                clip_param=0.2, entcoeff=0.0,
                optim_epochs=10, 
                optim_stepsize=3e-4, 
                optim_batchsize=64,
                gamma=0.995, lam=0.95, schedule='linear')

    saver  = tf.train.Saver()
    saver.restore(tf.get_default_session(), model_directory + model_filename)
    print("loaded")

    W0, b0, W_hidden, b_hidden, W1, b1, ob_mean, ob_std = pposgd_simple.get_policy_parameters(pi)

    fg = firmware_generator.FirmwareGenerator(W0 = W0, b0 = b0, W_hidden = W_hidden, b_hidden = b_hidden, \
                W1 = W1, b1 = b1, num_hidden_layer = len(W_hidden) + 1, ob_space_size = W0.shape[0], \
                hidden_layer_size = W0.shape[1], ac_space_size = W1.shape[1], state_size = 9, \
                final_bias = 3.5, ob_mean = ob_mean, ob_std = ob_std)
    fg.generate()

    # Test weights
    _iter = 0
    ob = env.reset()
    while True:
        print(ob)
        ob = (ob - ob_mean) / ob_std
        ob = np.clip(ob, -5.0, 5.0)
        print("final ob = ", ob)
        last_out = np.tanh(ob.dot(W0) + b0)
        for j in range(len(W_hidden)):
            last_out = np.tanh(last_out.dot(W_hidden[j]) + b_hidden[j])
        ac = last_out.dot(W1) + b1
        ob, _, _, _ = env.step(ac)
        _iter += 1

    env.close()

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--controller', type = str, default = "")

    args = parser.parse_args()
    controller = args.controller

    logger.configure()
    save(model_directory="./results/",
        model_filename=controller + ".ckpt"
    )
    
if __name__=='__main__':
    main()