#pragma once
#include <Eigen/Dense>
#include "CopterUtils.h"
#include "CopterSimulationCommon.h"
#include "CopterMesh.h"
#include "Aerodynamics/AerodynamicsDatabase.h"
#include "Rotor/Rotor.h"
#include "RigidBody/RigidBody.h"
#include "Sensor/InertialSensor.h"
#include "Controller/PID/PIDAltHoldController.h"
#include "Controller/PID/PIDQuadPlaneController.h"
#include "Controller/NN/NNController.h"
#include "Log.h"

namespace copter_simulation {

class HybridCopter
{
public:
    typedef copter_utils::RandomGenerator Random;

    HybridCopter(const int simulation_fps = 25.0);
    ~HybridCopter();

    void Initialize();
    void InitializeControllers();

    void SwitchController();

    /* Simulation Functions */
    void Reset();
    void Advance(float dt);
    void RunSimulation(Eigen::VectorXf& motor_output, float dt);
    void UpdateAerodynamics(RigidBody& body);

    /* Radio Communication */
    void SetRadioInput(Eigen::Vector4i rpyt_in, int mode_switch);

    /* Basic Tools */
    void ComputeMassProperty(float& mass, Eigen::Matrix3f& inertia_body);

    const Eigen::Matrix4f TransformBodyToWorld(int component_id) const;

    bool LoadFromConfigFile(std::string config_file_name);

    bool LoadScript(const std::string &script_file_name);

    /* Set/Get Functions */
    CopterMesh& copter_mesh() { return _copter_mesh; }
    RigidBody& rigid_body() { return _rigid_body; }
    int simulation_fps() { return _simulation_fps; }
    std::vector<Rotor>& rotors() { return _rotors; }
    const std::vector<Rotor>& rotors() const { return _rotors; }
    Rotor& rotors(int idx) { return _rotors[idx]; }
    int rotor_num() { return _rotors.size(); }
    InertialSensor& clean_sensor() { return _clean_sensor; } 
    InertialSensor& sensor() { return _sensor; }
    float angle_of_attack() { return _angle_of_attack; }
    Eigen::Vector3f lift_force() { return _lift_force; }
    Eigen::Vector3f drag_force() { return _drag_force; }
    FlightController flight_controller() { return _flight_controller; }
    FlightMode flight_mode() { return _flight_mode; }
    float roll_in() { return _roll_in; }
    float pitch_in() { return _pitch_in; }
    float yaw_in() { return _yaw_in; }
    float throttle_in() { return _throttle_in; }
    float target_yaw_diff() { return _target_yaw_diff; }
    float vx_in() { return _vx_in; }
    float vy_in() { return _vy_in; }
    float vz_in() { return _vz_in; }
    float current_time() { return _current_time; }
    float last_action(int idx) const { return _last_action[idx]; }
    bool director_mode() const { return _director_mode; }
    Eigen::Vector3f current_target() { return _script_step > 0 ? Eigen::Vector3f(_script_targets[_script_step - 1].head(3)) : Eigen::Vector3f::Zero(); }

private:
    // Utility function for safe start
    float SafeStart(float throttle_in, float throttle_min, float throttle_max, bool throttle_control = true);

    CopterMesh _copter_mesh;
    std::vector<Rotor> _rotors;
    AerodynamicsDatabase _aero_database;

    // Noise
    bool _noisy_body = true;
    bool _noisy_sensor = true;
    bool _noisy_rotor = true;
    bool _simulate_wind = false;
    bool _simulate_delay = false;
    bool _noisy_dt = false;
    bool _constrain_motor_output = false;
    bool _noisy_motor_position = false;
    bool _noisy_motor_direction = false;
    bool _noisy_fuselage_drag = false;

    float _noise_mass = 0.05f;
    float _noise_inertia = 0.4f;
    float _noise_dt = 0.3f;
    float _delay = 0.04f;

    std::vector<float> _last_action;

    // Delay table
    std::vector<Eigen::Vector4f> _rpyt_his;
    std::vector<float> _time_his;

    // Taking off
    bool _taking_off;
    float _taking_off_height;

    // PID controller signal
    float _roll_in, _pitch_in, _yaw_in, _throttle_in;
    bool _throttle_activated;

    // NN controller signal
    float _vx_in, _vy_in, _vz_in;
    float _target_roll_diff, _target_yaw_diff;
    
    // for simulation
    int _simulation_fps;
    float _current_time;
    RigidBody _rigid_body;

    // for Aerodynamics
    Eigen::Vector3f _wing_direction;
    Eigen::Vector3f _aerodynamic_center;
    Eigen::Vector3f _lift_force, _drag_force;
    float _angle_of_attack;

    // Sensors
    InertialSensor _clean_sensor;   // Clean sensor for GUI display
    InertialSensor _sensor;         // Noisy (or clean) sensor for controller

    // controllers
    FlightController _flight_controller;
    PIDQuadPlaneController _pid_quadplane_controller;
    NNController _nn_controller;

    // variables for controller
    FlightMode _flight_mode;
    bool _in_transition;

    // Battery
    float _battery_mass;
    Eigen::Vector3f _battery_pos;

    // Log
    copter_utils::Log _log;

    // Flight Log
    std::vector<Eigen::Vector4f> _flight_log;
    int timestep;

    // Director mode
    bool _director_mode;
    int _script_step;
    std::vector<Eigen::Vector4f> _script_targets;
    std::vector<float> _script_timestamps;
};

}