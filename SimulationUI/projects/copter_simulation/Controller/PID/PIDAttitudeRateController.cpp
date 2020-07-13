#include "PIDAttitudeRateController.h"
#include "HybridCopter.h"

namespace copter_simulation {

PIDAttitudeRateController::PIDAttitudeRateController(
        InertialSensor& sensor, HybridCopter& copter)
    : _sensor(sensor), _copter(copter),
    _roll_rate_pid(RATE_RP_P, RATE_RP_I, RATE_RP_D, RATE_RP_IMAX, RATE_RP_FILT_HZ, 1.0f / _copter.simulation_fps()),
    _pitch_rate_pid(RATE_RP_P, RATE_RP_I, RATE_RP_D, RATE_RP_IMAX, RATE_RP_FILT_HZ, 1.0f / _copter.simulation_fps()),
    _yaw_rate_pid(RATE_YAW_P, RATE_YAW_I, RATE_YAW_D, RATE_YAW_IMAX, RATE_YAW_FILT_HZ, 1.0f / _copter.simulation_fps()) {}

void PIDAttitudeRateController::reset() {
    _roll_rate_pid.reset();
    _pitch_rate_pid.reset();
    _yaw_rate_pid.reset();
}

void PIDAttitudeRateController::output(
    const Eigen::Vector3f& target_euler_rate_roll_pitch_yaw, 
    Eigen::Vector3f& roll_pitch_yaw_output) {

    Eigen::Vector3f target_rpy_rate = target_euler_rate_roll_pitch_yaw;
    Eigen::Vector3f actual_rpy_rate = _sensor.rpy_rate();
    roll_pitch_yaw_output = Eigen::Vector3f(
        compute_pid(_roll_rate_pid, actual_rpy_rate(0), target_rpy_rate(0)),
        compute_pid(_pitch_rate_pid, actual_rpy_rate(1), target_rpy_rate(1)),
        compute_pid(_yaw_rate_pid, actual_rpy_rate(2), target_rpy_rate(2)));
}

float PIDAttitudeRateController::compute_pid(
    PID& pid, float actual, float target) {
    
    float error = target - actual;

    pid.set_input_filter_d(error);
    
    return copter_utils::Clamp(pid.get_pid(), -1.0f, 1.0f);
}

}