#include "HybridCopter.h"
#include "tiny_obj_loader.h"
#include "Controller/NN/NNController.h"
#include <random>

namespace copter_simulation {

HybridCopter::HybridCopter(const int simulation_fps)
    : _simulation_fps(simulation_fps),
    _taking_off(true), _taking_off_height(0.0f),
    _in_transition(false), _flight_mode(FlightMode::COPTER_MODE),
    _clean_sensor(false),
    _sensor(_noisy_sensor, _simulate_delay, _delay),
    _pid_quadplane_controller(_sensor, *this),
    _nn_controller(_sensor, *this),
    _roll_in(0.0f), _pitch_in(0.0f), _yaw_in(0.0f),
    _throttle_in(0.0f), _throttle_activated(false),
    _log(k_param_project_source_dir + k_param_log_directory),
    _flight_controller(FlightController::NN),
    _director_mode(false), _script_step(0) {}


HybridCopter::~HybridCopter()
{
}

void HybridCopter::Initialize() {
    Reset();

    InitializeControllers();
}

void HybridCopter::SwitchController() {
    if (_flight_controller == FlightController::NN)
        _flight_controller = FlightController::QuadPlane;
    else if (_flight_controller == FlightController::QuadPlane)
        _flight_controller = FlightController::NN;
}

void HybridCopter::InitializeControllers() {
    _pid_quadplane_controller.add_motors(_rotors);
    assert((int)_rotors.size() == _nn_controller.action_space_dim());
}

void HybridCopter::Reset() {
    _current_time = 0.0f;

    /* Reset Rigid Body */
    float mass;
    Eigen::Matrix3f inertia_body;

    ComputeMassProperty(mass, inertia_body);

    // Add noise to mass and inertia tensor
    if (_noisy_body) {
        mass *= Random::uniform<float>(1.0f - _noise_mass, 1.0f + _noise_mass);
        for (int i = 0; i < 3; ++i)
            for (int j = i; j < 3; ++j) {
                inertia_body(i, j) *= Random::uniform<float>(1.0f - _noise_inertia, 1.0f + _noise_inertia);
                inertia_body(j, i) = inertia_body(i, j);
            }
    }

    // Taking-off stage
    const auto&& right = _copter_mesh.BoundingBox()._right_corner;
    _taking_off_height = -right(2);
    _taking_off = true;

    _rigid_body.Initialize(mass, inertia_body.cast<double>(),
        Eigen::Vector3d::UnitZ() * _taking_off_height,
        Eigen::Vector3d::Zero(),
        Eigen::Quaterniond::Identity(),
        Eigen::Vector3d::Zero());

    /* Reset signal input */
    _roll_in = _pitch_in = _yaw_in = _throttle_in = 0.0f;
    _throttle_activated = false;

    _target_yaw_diff = 0.0;

    /* Reset Sensors */
    _clean_sensor.Reset();
    _sensor.Reset();
    _clean_sensor.Update(_rigid_body.orientation().cast<float>(), _rigid_body.omega().cast<float>(),
        _rigid_body.position().cast<float>(), _rigid_body.velocity().cast<float>(), _rigid_body.acceleration().cast<float>(), 0.0);
    _sensor.Update(_rigid_body.orientation().cast<float>(), _rigid_body.omega().cast<float>(),
        _rigid_body.position().cast<float>(), _rigid_body.velocity().cast<float>(), _rigid_body.acceleration().cast<float>(), 0.0);

    /* Reset Controllers */
    _pid_quadplane_controller.reset();
    _nn_controller.reset();

    // Reset delay queues
    _rpyt_his.clear();
    _rpyt_his.reserve(1 << 16);
    _time_his.clear();
    _time_his.reserve(1 << 16);

    // reset last action
    _last_action.clear();
    _last_action.resize(_rotors.size(), 0.0f);

    // Beichen Li: load script for director's mode
    if (_director_mode)
        LoadScript(k_param_project_source_dir + k_param_script_path);
}

void HybridCopter::Advance(float dt) {
    // run controller
    Eigen::Vector4f rpyt_in;
    Eigen::VectorXf motor_output;

    // Beichen Li: director's mode (restricted to NN controller)
    if (_director_mode) { 
        assert(_flight_controller == FlightController::NN);

        // Switch to the next target
        if (_script_step < (int)_script_timestamps.size() && _script_timestamps[_script_step] <= _current_time) {
            std::cout << "Script step " << _script_step << ": target = " <<
                _script_targets[_script_step].transpose().eval() << std::endl;
            _script_step++;
        }
        // Deploy the current target
        if (_script_step > 0) {
            auto current_target = _script_targets[_script_step - 1];
            // Smooth transition
            if (_script_step < (int)_script_timestamps.size()) {
                auto next_target = _script_targets[_script_step];
                float transition = std::min(_script_timestamps[_script_step] - _script_timestamps[_script_step - 1], k_param_max_transition);
                float time_left = _script_timestamps[_script_step] - _current_time;
                if (time_left <= transition)
                    current_target = (current_target * time_left + next_target * (transition - time_left)) / transition;
            }
            _vx_in = current_target[0];
            _vy_in = current_target[1];
            _vz_in = current_target[2];
            _target_yaw_diff = current_target[3];
        }
    }

    // add record to delay table
    if (_flight_controller == FlightController::QuadPlane) {
        _rpyt_his.emplace_back(Eigen::Vector4f(_roll_in, _pitch_in, _yaw_in, _throttle_in));
    }
    else if (_flight_controller == FlightController::NN) {
        _rpyt_his.emplace_back(Eigen::Vector4f(_vx_in, _vy_in, _vz_in, _target_yaw_diff));
    }
    _time_his.push_back(_current_time);

    // delayed output
    // int his_idx = _rpyt_his.size() - 1;
    // while (his_idx > 0 && _time_his[his_idx] > _current_time - _delay)
    //     --his_idx;
    // if (_flight_controller == FlightController::QuadPlane) {
    //     _pid_quadplane_controller.output(_rpyt_his[his_idx], motor_output);
    // }
    // else if (_flight_controller == FlightController::NN) {
    //     _nn_controller.output(_rpyt_his[his_idx], motor_output);
    // }

    if (_flight_controller == FlightController::QuadPlane) {
        rpyt_in = Eigen::Vector4f(_roll_in, _pitch_in, _yaw_in, _throttle_in);
        _pid_quadplane_controller.output(rpyt_in, motor_output);
    } else if (_flight_controller == FlightController::NN) {
        rpyt_in = Eigen::Vector4f(_vx_in, _vy_in, _vz_in, _target_yaw_diff);
        _nn_controller.output(rpyt_in, motor_output);
    }

    // constrain motor output
    if (_constrain_motor_output) {
        for (int i = 0;i < _rotors.size();i++) {
            motor_output[i] = copter_utils::Clamp(motor_output[i], _last_action[i] - 0.2, _last_action[i] + 0.2);
        }
    }

    // record action
    for (int i = 0; i < (int)_rotors.size(); ++i)
        _last_action[i] = motor_output[i];

    // simulation
    dt = _noisy_dt ? Random::uniform<float>(1.0f - _noise_dt, 1.0f + _noise_dt) * dt : dt;
    RunSimulation(motor_output, dt);
    _current_time += dt;

    // update clean sensor
    _clean_sensor.Update(_rigid_body.orientation().cast<float>(), _rigid_body.omega().cast<float>(),
        _rigid_body.position().cast<float>(), _rigid_body.velocity().cast<float>(), _rigid_body.acceleration().cast<float>(), _current_time);
    
    // update controller sensor
    _sensor.Update(_rigid_body.orientation().cast<float>(), _rigid_body.omega().cast<float>(),
        _rigid_body.position().cast<float>(), _rigid_body.velocity().cast<float>(), _rigid_body.acceleration().cast<float>(), _current_time);

    // Log
    _log.Buffer(_current_time);
    _log.Buffer(_clean_sensor.position().cast<double>());
    _log.Buffer(_clean_sensor.velocity().cast<double>());
    _log.Buffer(_clean_sensor.acceleration().cast<double>());
    _log.Buffer(_clean_sensor.rpy().cast<double>());
    _log.Buffer(_clean_sensor.rpy_rate().cast<double>());
    _log.Buffer(rpyt_in.cast<double>());
    for (int i = 0;i < _rotors.size();++i)
        _log.Buffer(_rotors[i].desired_thrust());
    _log.Flush();
}

void HybridCopter::RunSimulation(Eigen::VectorXf& motor_output, float dt) {
    // Apply Gravity
    _rigid_body.ApplyForceInWorldFrame(
        Eigen::Vector3d::UnitZ() * _rigid_body.mass() * k_param_gravity,
        _rigid_body.PointBodyToWorld(Eigen::Vector3d(0.0f, 0.0f, 0.0f)));
 
    // Apply Thrust
    for (int i = 0;i < _rotors.size();++i)
        _rotors[i].output(motor_output[i]);
    for (int i = 0;i < _rotors.size();++i) {
        _rigid_body.ApplyForceInBodyFrame(_rotors[i].get_thrust().cast<double>(),
            _rotors[i].position_body_frame().cast<double>());
        _rigid_body.ApplyTorque(
            _rigid_body.VectorBodyToWorld(_rotors[i].get_torque().cast<double>()));
    }

    // Apply Aerodynamic Forces
    UpdateAerodynamics(_rigid_body);
    _rigid_body.ApplyForceInWorldFrame(
        _lift_force.cast<double>(),
        _rigid_body.PointBodyToWorld(_aerodynamic_center.cast<double>()));
    _rigid_body.ApplyForceInWorldFrame(
        _drag_force.cast<double>(),
        _rigid_body.PointBodyToWorld(_aerodynamic_center.cast<double>()));

    // Add Fuselage Drag
    if (_noisy_fuselage_drag) {
        float coef = Random::normal<float>(0.02, 0.01);
        _rigid_body.ApplyForceInWorldFrame(
            -coef * _rigid_body.velocity().norm() * k_param_air_rho * _rigid_body.velocity(),
            _rigid_body.PointBodyToWorld(Eigen::Vector3d(0.0f, 0.0f, 0.0f)));
    }
    
    // Taking-off stage
    if (_rigid_body.external_force()(2) < 0)
        _taking_off = false;
    if (_rigid_body.position()(2) > _taking_off_height) {
        _rigid_body.TouchGround(_taking_off_height);
        _taking_off = true;
    }
    if (_taking_off)
        _rigid_body.Clear();

    _rigid_body.Advance(dt);
}

void HybridCopter::UpdateAerodynamics(RigidBody& body) {
    Eigen::Vector3d chord_direction = _wing_direction.cast<double>();
    Eigen::Vector3d downward_direction(-_wing_direction[2], _wing_direction[1], _wing_direction[0]);

    chord_direction.normalize(); downward_direction.normalize();
    
    Eigen::Vector3d velocity_world = body.velocity();
    Eigen::Matrix3d R = body.orientation().matrix();
    Eigen::Vector3d velocity_body = R.transpose() * velocity_world;
    Eigen::Vector3d velocity_body_projected(velocity_body(0), 0.0f, velocity_body(2));

    if (velocity_body_projected.norm() < eps) {
        _angle_of_attack = 0.0f;
        _lift_force << 0, 0, 0;
        _drag_force << 0, 0, 0;
        return;
    }

    Eigen::Vector3d lift_dir_body(velocity_body(2), 0.0f, -velocity_body(0));
    Eigen::Vector3d drag_dir_body = -velocity_body;
    lift_dir_body.normalize();
    drag_dir_body.normalize();

    Eigen::Vector3f lift_dir = body.VectorBodyToWorld(lift_dir_body).cast<float>();
    Eigen::Vector3f drag_dir = body.VectorBodyToWorld(drag_dir_body).cast<float>();

    _angle_of_attack = std::asin(velocity_body_projected.dot(downward_direction) / velocity_body_projected.norm());

    _lift_force = _aero_database.get_lift(_angle_of_attack, velocity_body_projected.norm()) * lift_dir;
    _drag_force = _aero_database.get_drag(_angle_of_attack, velocity_body_projected.norm()) * drag_dir;

    _aerodynamic_center = _aero_database.aerodynamic_center();
}

const Eigen::Matrix4f HybridCopter::TransformBodyToWorld(int component_id) const {
    return _rigid_body.BodyToWorldTransform().cast<float>();
}

float HybridCopter::SafeStart(float throttle_in, float throttle_min, float throttle_max, bool throttle_control) {
    // The default mode is COPTER_MODE, which works for all controllers
    if (_flight_mode == FlightMode::COPTER_MODE) {
        if (throttle_control && throttle_in >= k_param_radio_trim)
            _throttle_activated = true;
        if (throttle_control && !_throttle_activated)
            return 0.0;
        if (throttle_in < k_param_radio_trim - k_param_radio_deadzone)
            return copter_utils::Remap(throttle_in,
                k_param_radio_min, k_param_radio_trim - k_param_radio_deadzone,
                throttle_min, 0.0);
        else if (throttle_in > k_param_radio_trim + k_param_radio_deadzone)
            return copter_utils::Remap(throttle_in,
                k_param_radio_trim + k_param_radio_deadzone, k_param_radio_max,
                0.0, throttle_max);
        else
            return 0.0;
    }
    else if (_flight_mode == FlightMode::GLIDING_MODE) {
        if (throttle_control && throttle_in >= (k_param_radio_trim + k_param_radio_max) / 2.0f)
            _throttle_activated = true;
        if (throttle_control && !_throttle_activated) {
            if (throttle_in < k_param_radio_trim)
                return 0.0f;
            else
                return copter_utils::Remap(throttle_in,
                    k_param_radio_trim, k_param_radio_max,
                    0.0f, throttle_max);
        } else
            return copter_utils::Remap(throttle_in,
                k_param_radio_min, k_param_radio_max,
                0.0f, throttle_max);
    }
}

void HybridCopter::SetRadioInput(Eigen::Vector4i rpyt_in, int mode_switch) {
    if (_flight_controller == FlightController::QuadPlane) {
        // handle roll, pitch, yaw, throttle input
        float roll_in = (float)rpyt_in[0];
        float pitch_in = (float)rpyt_in[1];
        float yaw_in = (float)rpyt_in[2];
        float throttle_in = (float)rpyt_in[3];
        _roll_in = copter_utils::Remap(roll_in, k_param_radio_min, k_param_radio_max,
            copter_utils::degree2radian(k_param_roll_pitch_angle_min),
            copter_utils::degree2radian(k_param_roll_pitch_angle_max));
        if (_flight_mode == FlightMode::COPTER_MODE) {
            _pitch_in = copter_utils::Remap(pitch_in, k_param_radio_min, k_param_radio_max,
                copter_utils::degree2radian(k_param_roll_pitch_angle_min),
                copter_utils::degree2radian(k_param_roll_pitch_angle_max));
            _yaw_in = copter_utils::Remap(yaw_in, k_param_radio_min, k_param_radio_max,
                k_param_yaw_rate_min, k_param_yaw_rate_max);
        }
        else if (_flight_mode == FlightMode::GLIDING_MODE) {
            if (_throttle_activated)
                _pitch_in = copter_utils::degree2radian(10);
            else
                _pitch_in = copter_utils::Remap(pitch_in, k_param_radio_min, k_param_radio_max,
                    copter_utils::degree2radian(k_param_roll_pitch_angle_min),
                    copter_utils::degree2radian(k_param_roll_pitch_angle_max));
            _yaw_in = _roll_in * 2.0;
        }
        _throttle_in = SafeStart(throttle_in, k_param_altitude_rate_min, k_param_altitude_rate_max);

        // handle mode switch
        FlightMode new_flight_mode;
        if (mode_switch == 0) new_flight_mode = FlightMode::COPTER_MODE;
        else new_flight_mode = FlightMode::GLIDING_MODE;

        _pid_quadplane_controller.set_mode(_flight_mode);

        if (new_flight_mode != _flight_mode) {
            _flight_mode = new_flight_mode;
            _in_transition = true;
            _throttle_activated = false;
        }
    }
    else if (_flight_controller == FlightController::NN) {
        // handle vx, vy, vz and yaw input

        _vy_in = 0.0f;
        // _vx_in = -std::min(SafeStart((float)rpyt_in(1), k_param_vx_min, k_param_vx_max, false), 0.0f);
        if (rpyt_in(1) > k_param_radio_trim) {
            _vx_in = copter_utils::Remap((float)rpyt_in(1), k_param_radio_trim, k_param_radio_max, 0.0, k_param_vx_min);
        } else {
            _vx_in = copter_utils::Remap((float)rpyt_in(1), k_param_radio_min, k_param_radio_trim, k_param_vx_max, 0.0);
        }

        _yaw_in = 0.0f;
        _vz_in = copter_utils::Remap((float)rpyt_in(3), k_param_radio_min, k_param_radio_max, k_param_vz_max, k_param_vz_min);
        _vy_in = copter_utils::Remap((float)rpyt_in(0), k_param_radio_min, k_param_radio_max, k_param_vy_min, k_param_vy_max);

        _target_yaw_diff += copter_utils::Remap((float)rpyt_in(2), k_param_radio_min, k_param_radio_max, -1.0, 1.0) * 0.0025;
    }
}

void HybridCopter::ComputeMassProperty(float& mass, Eigen::Matrix3f& inertia_body) {
    // assume the COM is at origin

    /* compute mass */
    mass = 0.0;
    // add copter mesh mass
    auto body_mesh_mass = _copter_mesh.body_mesh_mass();
    for (int i = 0;i < body_mesh_mass.size();++i)
        mass += body_mesh_mass[i];
    // add motor mass
    for (int i = 0;i < _rotors.size();++i)
        mass += _rotors[i].mass();
    // add battery mass
    mass += _battery_mass;

    /* compute inertia */
    inertia_body.setZero();
    // add copter mesh inertia
    auto body_mesh = _copter_mesh.body_mesh();
    for (int i = 0;i < body_mesh.size();++i) {
        double submass = body_mesh_mass[i];
        std::vector<Eigen::Vector3f>& vertices = body_mesh[i].vertices();
        for (int j = 0;j < vertices.size();++j) {
            const Eigen::Vector3f r = vertices[j];
            const Eigen::Matrix3f skew_r = copter_utils::SkewMatrix(r);
            inertia_body -= skew_r * skew_r * (submass / (float)vertices.size());
        }
    }
    // add motor inertia
    for (int i = 0;i < _rotors.size();++i) {
        const Eigen::Vector3f r = _rotors[i].position_body_frame();
        const Eigen::Matrix3f skew_r = copter_utils::SkewMatrix(r);
        inertia_body -= skew_r * skew_r * _rotors[i].mass();
    }
    // add battery inertia
    {
        const Eigen::Matrix3f skew_r = copter_utils::SkewMatrix(_battery_pos);
        inertia_body -= skew_r * skew_r * _battery_mass;
    }
}

bool HybridCopter::LoadFromConfigFile(std::string config_file_name) {
#ifndef QUADROTOR_DEBUG
    // The center of mass should lie at origin

    // The config file format:
    // Line 1: obj parts directory
    // Line 2: The scale factor(double).
    // Line 3: Wing direction
    // Line 4: Wing area
    // Line 5: angle of attack 0
    // Line 6: aerodynamic center
    // Line 7: The number of propellers N(int).
    // Line 8 - N + 7: Each line contains 7 double numbers (dx, dy, dz, x, y, z,
    // is_cw, weight), where (dx, dy, dz) is the orientation of the propeller, (x, y, z)
    // is the position, is_cw is boolean indicating if propeller spin clockwise, 
    // weight is the mass of the motor+propeller

    // The part_list file format:
    // each line: mesh file name, mesh name, mesh mass
    
    std::string config_file_directory = copter_utils::get_parent_directory(config_file_name);

    CopterMesh copter_mesh;
    std::vector<std::string> mesh_names;
    Eigen::Vector3f wing_direction;
    float wing_area, angle_of_attack0;
    Eigen::Vector3f aerodynamic_center;
    float battery_mass;
    Eigen::Vector3f battery_pos;
    std::vector<Eigen::Vector3f> rotor_positions;
    std::vector<Eigen::Vector3f> rotor_directions;
    std::vector<int> rotor_spin_directions;
    std::vector<float> rotor_mass;
    
    std::ifstream config_file(config_file_name);
    if (config_file.fail())
        return false;

    
    {
        mesh_names.clear();
        std::string mesh_directory;
        config_file >> mesh_directory;
        std::ifstream part_list_file(config_file_directory + mesh_directory + "/part_list.txt");
        std::string mesh_file_name, mesh_name;
        float mass;
        while (part_list_file >> mesh_file_name >> mesh_name >> mass) {
            std::string mesh_file_dir = config_file_directory + mesh_directory + "/" + mesh_file_name + ".obj";
            mesh_names.push_back(mesh_name);
            copter_mesh.body_mesh_mass().push_back(mass);
            std::vector<tinyobj::shape_t> obj_shape;
            std::vector<tinyobj::material_t> obj_material;
            tinyobj::attrib_t attrib;
            std::string err;
            tinyobj::LoadObj(&attrib, &obj_shape, &obj_material, &err, mesh_file_dir.c_str());
            copter_utils::SurfaceMesh part_mesh;
            for (int j = 0;j < (int)attrib.vertices.size() / 3;++j) {
                part_mesh.vertices().push_back(Eigen::Vector3f(attrib.vertices[j * 3],
                    attrib.vertices[j * 3 + 1],
                    attrib.vertices[j * 3 + 2]));
            }
            for (int j = 0;j < (int)obj_shape[0].mesh.indices.size() / 3;++j) {
                part_mesh.elements().push_back(Eigen::Vector3i(obj_shape[0].mesh.indices[j * 3].vertex_index,
                    obj_shape[0].mesh.indices[j * 3 + 1].vertex_index,
                    obj_shape[0].mesh.indices[j * 3 + 2].vertex_index));
            }
            copter_mesh.body_mesh().push_back(part_mesh);
        }
        part_list_file.close();
    }
    
    // scale the meshes
    float scale_factor;
    {
        config_file >> scale_factor;
        copter_mesh.Scale(scale_factor);
    }
    
    // load wing info
    {
        config_file >> wing_direction[0] >> wing_direction[1] >> wing_direction[2];
        config_file >> wing_area;
        config_file >> angle_of_attack0;
        config_file >> aerodynamic_center[0] >> aerodynamic_center[1] >> aerodynamic_center[2];
        aerodynamic_center *= scale_factor;
    }
    
    // load battery info
    {
        config_file >> battery_mass;
        config_file >> battery_pos(0) >> battery_pos(1) >> battery_pos(2);
    }

    // load propeller info
    {
        rotor_positions.clear();
        rotor_directions.clear();
        rotor_spin_directions.clear();
        int propeller_num;
        config_file >> propeller_num;
        for (int i = 0;i < propeller_num;++i) {
            Eigen::Vector3f pos;
            Eigen::Vector3f dir;
            int spin_direction;
            float _rotor_mass;

            config_file >> pos(0) >> pos(1) >> pos(2);
            config_file >> dir(0) >> dir(1) >> dir(2);
            config_file >> spin_direction;
            config_file >> _rotor_mass;
            pos *= scale_factor;
            rotor_positions.push_back(pos);
            rotor_directions.push_back(dir);
            rotor_spin_directions.push_back(spin_direction);
            rotor_mass.push_back(_rotor_mass);
        }
    }
    
    // Translate so that its center of mass is at the origin.
    {
        // Compute COM first
        Eigen::Vector3f copter_torque(0.0f, 0.0f, 0.0f);
        copter_torque += copter_mesh.Center() * copter_mesh.Mass();
        copter_torque += battery_pos * battery_mass;
        for (int i = 0; i < (int)rotor_positions.size(); ++i)
            copter_torque += rotor_positions[i] * rotor_mass[i];
        float copter_mass = copter_mesh.Mass() + battery_mass;
        for (auto &mass : rotor_mass)
            copter_mass += mass;
        auto frame_center = copter_torque / copter_mass;
        copter_mesh.Translate(-frame_center);
        for (int i = 0;i < rotor_positions.size();++i)
            rotor_positions[i] -= frame_center;
        aerodynamic_center -= frame_center;
        battery_pos -= frame_center;
    }
    
    config_file.close();
    
    // copy data to store
    _copter_mesh = copter_mesh;

    if (_noisy_motor_direction) {
        for (int i = 0;i < rotor_directions.size();i++) {
            float angle_x = Random::normal<float>(0.0, 0.07);
            rotor_directions[i] = Eigen::AngleAxisf(angle_x, Eigen::Vector3f(1, 0, 0)) * rotor_directions[i];
            std::cerr << "noisy angle_x = " << angle_x / PI * 180.0 << std::endl;
        }
        float angle_y = Random::normal<float>(0.0, 0.1);
        for (int i = 0;i < rotor_directions.size();i++) {
            rotor_directions[i] = Eigen::AngleAxisf(angle_y, Eigen::Vector3f(0, 1, 0)) * rotor_directions[i];
        }
        std::cerr << "common angle_y = " << angle_y / PI * 180.0 << std::endl;
        // for (int i = 0;i < rotor_directions.size();i++) {
        //     if (i == 0 || i == 1) {
        //         float angle_y = Random::normal<float>(0.0, 0.01);
        //         rotor_directions[i] = Eigen::AngleAxisf(angle_y, Eigen::Vector3f(0, 1, 0)) * rotor_directions[i];
        //         std::cerr << "noisy angle_y = " << angle_y / PI * 180.0 << std::endl;
        //     }
        //     if (i == 2) {
        //         rotor_directions[i] = rotor_directions[1];
        //     }
        //     if (i == 3) {
        //         rotor_directions[i] = rotor_directions[0];
        //     }
        // }
    }
    for (int i = 0;i < rotor_positions.size();++i) {
        if (_noisy_motor_position) {
            for (int j = 0;j < 3;++j)
                rotor_positions[i](j) = rotor_positions[i](j) + Random::normal<float>(0.0, 0.02);
        }
        _rotors.push_back(Rotor(rotor_directions[i], rotor_positions[i],
            rotor_spin_directions[i] == 0, rotor_mass[i]));
    }
    
    _aero_database.Initialize(angle_of_attack0, wing_area, 0.0, aerodynamic_center);

    _wing_direction = wing_direction;

    _battery_mass = battery_mass;
    _battery_pos = battery_pos;

    return true;
#else
    CopterMesh copter_mesh;
    std::vector<std::string> mesh_names;
    std::string config_file_directory = copter_utils::get_parent_directory(config_file_name);
    mesh_names.clear();
    std::string mesh_file_dir = config_file_directory + "DJI_F450/F450_course.obj";
    mesh_names.push_back("F450");
    copter_mesh.body_mesh_mass().push_back(1.0);
    std::vector<tinyobj::shape_t> obj_shape;
    std::vector<tinyobj::material_t> obj_material;
    tinyobj::attrib_t attrib;
    std::string err;
    tinyobj::LoadObj(&attrib, &obj_shape, &obj_material, &err, mesh_file_dir.c_str());
    copter_utils::SurfaceMesh part_mesh;
    for (int j = 0;j < (int)attrib.vertices.size() / 3;++j) {
        part_mesh.vertices().push_back(Eigen::Vector3f(attrib.vertices[j * 3],
            -attrib.vertices[j * 3 + 1],
            -attrib.vertices[j * 3 + 2]));
    }
    for (int j = 0;j < (int)obj_shape[0].mesh.indices.size() / 3;++j) {
        part_mesh.elements().push_back(Eigen::Vector3i(obj_shape[0].mesh.indices[j * 3].vertex_index,
            obj_shape[0].mesh.indices[j * 3 + 1].vertex_index,
            obj_shape[0].mesh.indices[j * 3 + 2].vertex_index));
    }
    copter_mesh.body_mesh().push_back(part_mesh);
    _copter_mesh = copter_mesh;

    _rotors.clear();
    _rotors.push_back(Rotor(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(0.16251, 0.16249, -0.03587), 1, 0.0));
    _rotors.push_back(Rotor(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(-0.16249, -0.16251, -0.03587), 1, 0.0));
    _rotors.push_back(Rotor(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(0.16251, -0.16251, -0.03587), 0, 0.0));
    _rotors.push_back(Rotor(Eigen::Vector3f(0, 0, -1), Eigen::Vector3f(-0.16249, 0.16249, -0.03587), 0, 0.0));
    _aero_database.Initialize(0, 0, 0.0, Eigen::Vector3f(0, 0, 0));
#endif
}

bool HybridCopter::LoadScript(const std::string &script_file_name) {
    // Read from script file
    std::ifstream ifs(script_file_name);
    assert(ifs.is_open());

    _script_targets.clear();
    _script_timestamps.clear();
    float t = 0.0f, dt, vx, vy, vz, yaw;
    while (ifs >> dt >> vx >> vy >> vz >> yaw) {
        _script_targets.emplace_back(Eigen::Vector4f(vx, vy, vz, yaw));
        _script_timestamps.push_back(t += dt);
    }
    _script_step = 0;
    
    return true;
}

}