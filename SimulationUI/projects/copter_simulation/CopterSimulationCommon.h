#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <fstream>
#include <Eigen/Dense>

#ifndef PI
    #define PI 3.1415927f
#endif

//#ifndef sqr(x)
//    #define sqr(x) ((x) * (x))
//#endif
// #define QUADROTOR_DEBUG

namespace copter_simulation {
    enum FlightMode {
        COPTER_MODE = 0,
        GLIDING_MODE = 1
    };

    enum FlightController {
        QuadPlane = 0,
        NN = 1
    };

    const float k_param_gravity = 9.8;
    const float k_param_air_rho = 1.225;

    // Rotor
    const float k_param_rotor_max_thrust = 7.0f;
    const float k_param_torque_to_thrust_ratio = 0.01554f;

    // Radio
    const int k_param_radio_trim = 1500;
    const int k_param_radio_min = 1000;
    const int k_param_radio_max = 2000;
    const int k_param_radio_deadzone = 100;
    const float k_param_roll_pitch_angle_min = -30.0f;
    const float k_param_roll_pitch_angle_max = 30.0f;
    const float k_param_yaw_rate_min = -PI / 4.0f;
    const float k_param_yaw_rate_max = PI / 4.0f;
    const float k_param_altitude_min = 0.0f;
    const float k_param_altitude_max = 7.0f;
    const float k_param_altitude_rate_min = -1.0f;
    const float k_param_altitude_rate_max = 1.0f;

// for E3
    // const float k_param_vx_min = -1.0f;
    // const float k_param_vx_max = 8.0f;
    // const float k_param_vy_min = -1.0f;
    // const float k_param_vy_max = 1.0f;
    // const float k_param_vz_min = -1.0f;
    // const float k_param_vz_max = 2.0f;

// for E4
    const float k_param_vx_min = -2.0f;
    const float k_param_vx_max = 6.0f;
    const float k_param_vy_min = -1.0f;
    const float k_param_vy_max = 1.0f;
    const float k_param_vz_min = -1.0f;
    const float k_param_vz_max = 1.0f;

    // PROJECT SOURCE DIR
    const std::string k_param_project_source_dir = std::string(PROJECT_SOURCE_DIR);
    
    // Log
    const std::string k_param_log_directory = "/data/SimulationLog/";

    // Script file
    const std::string k_param_script_path = "/data/config/script.txt";

    // Director's mode
    const float k_param_max_transition = 0.1f;

    // Scene file
    const std::string k_param_scene_path = "/data/config/scene.txt";
}
