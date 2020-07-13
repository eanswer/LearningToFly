#include "PIDAttitudeController.h"
#include "HybridCopter.h"

namespace copter_simulation {

PIDAttitudeController::PIDAttitudeController(
    InertialSensor& sensor, HybridCopter& copter)
    : _sensor(sensor), _copter(copter),
    _roll_pid(RP_P, RP_I, RP_D, RP_IMAX, RP_FILT_HZ, 1.0f / _copter.simulation_fps()),
    _pitch_pid(RP_P, RP_I, RP_D, RP_IMAX, RP_FILT_HZ, 1.0f / _copter.simulation_fps()) {}

void PIDAttitudeController::reset() {
    _roll_pid.reset();
    _pitch_pid.reset();
}

void PIDAttitudeController::output(
    const Eigen::Vector3f& target_euler_roll_pitch_euler_rate_yaw, 
    Eigen::Vector3f& target_euler_rate_roll_pitch_yaw) {
    
    float actual_roll = _sensor.roll();
    float actual_pitch = _sensor.pitch();
    
    float target_roll = target_euler_roll_pitch_euler_rate_yaw(0);
    float target_pitch = target_euler_roll_pitch_euler_rate_yaw(1);
    float target_yaw_rate = target_euler_roll_pitch_euler_rate_yaw(2);

    // std::cerr << "actual_roll = " << actual_roll << " , actual_pitch = " << actual_pitch << std::endl;
    target_euler_rate_roll_pitch_yaw(0) = compute_pid(_roll_pid, actual_roll, target_roll);
    target_euler_rate_roll_pitch_yaw(1) = compute_pid(_pitch_pid, actual_pitch, target_pitch);
    target_euler_rate_roll_pitch_yaw(2) = target_yaw_rate;
}

float PIDAttitudeController::compute_pid(
    PID& pid, float actual, float target) {
    float error = target - actual;

    pid.set_input_filter_all(error);

    return pid.get_pid();
}

}