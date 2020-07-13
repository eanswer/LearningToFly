#include "PIDPositionController.h"
#include "HybridCopter.h"

namespace copter_simulation {

    PIDPositionController::PIDPositionController(InertialSensor& sensor,
        HybridCopter& copter)
        : _sensor(sensor), _copter(copter),
        _throttle_hover(THROTTLE_HOVER),
        _pid_z_vel(POS_Z_VEL_P, POS_Z_VEL_I, POS_Z_VEL_D, POS_Z_VEL_IMAX, 2.0f, 1.0f / _copter.simulation_fps()),
        _pid_z_acc(POS_Z_ACC_P, POS_Z_ACC_I, POS_Z_ACC_D, POS_Z_ACC_IMAX, 2.0f, 1.0 / _copter.simulation_fps()) {
        _dt = 1.0f / _copter.simulation_fps();
    }

    void PIDPositionController::reset() {
        _target_z = _sensor.altitude();
        _target_z_vel = -_sensor.velocity().z();
        _target_z_acc = -_sensor.acceleration().z();
        _pid_z_vel.reset();
        _pid_z_acc.reset();
    }

    void PIDPositionController::output(float target_climb_rate,
        float& throttle_output) {

        // set_alt_target_from_climb_rate(target_climb_rate);

        // z_control(throttle_output);

        z_rate_control(target_climb_rate, throttle_output);
    }

    void PIDPositionController::set_alt_target_from_climb_rate(float climb_rate) {
        _target_z += climb_rate * _dt;
    }

    void PIDPositionController::z_control(float& throttle_output) {
    }

    void PIDPositionController::z_rate_control(float target_climb_rate,
        float& throttle_output) {

        float current_climb_rate = -_sensor.velocity().z();
        float error = target_climb_rate - current_climb_rate;

        _pid_z_vel.set_input_filter_all(error);
        float target_climb_acc = _pid_z_vel.get_pid();

        throttle_output = target_climb_acc;
        throttle_output += _throttle_hover;
        throttle_output = copter_utils::Clamp(throttle_output, 0.0f, 1.0f);
        // std::cerr << "actual_v_z = " << current_climb_rate << " , target_v_z = " << target_climb_rate << " , desired climb acc = " << target_climb_acc << std::endl;
 
        // z_acc_control(target_climb_acc, throttle_output);
    }

    void PIDPositionController::z_acc_control(float target_climb_acc,
        float& throttle_output) {

        float current_climb_acc = -_sensor.acceleration().z();
        float error = target_climb_acc - current_climb_acc;

        _pid_z_acc.set_input_filter_all(error);

        float throttle_compensate = _pid_z_acc.get_pid();
        
        throttle_output = throttle_compensate + _throttle_hover;
        
        // std::cerr << "actual_acc = " << current_climb_acc << " , target_acc = " << target_climb_acc << " , desired_throttle = " << throttle_output << std::endl;

        throttle_output = copter_utils::Clamp(throttle_output, 0.0f, 1.0f);
    }
}