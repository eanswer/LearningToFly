#include "PIDQuadPlaneController.h"
#include "HybridCopter.h"

namespace copter_simulation {

PIDQuadPlaneController::PIDQuadPlaneController(InertialSensor& sensor, HybridCopter& copter)
    : HighLevelController(sensor, copter),
    _althold_controller(sensor, copter) {
    
    _is_front_motor.clear();
    _motor_num = 0;
}

void PIDQuadPlaneController::reset() {
    _althold_controller.reset();
}

/* target front throttle is in [0, 1] */
void PIDQuadPlaneController::output(const Eigen::VectorXf& rpyt_in,
    Eigen::VectorXf& rpyt_out) {

    if (_mode == FlightMode::COPTER_MODE) {
        Eigen::VectorXf althold_input = (Eigen::VectorXf(4) << rpyt_in(0), rpyt_in(1),
            rpyt_in(2), rpyt_in(3)).finished();

        Eigen::VectorXf althold_output;

        _althold_controller.output(althold_input, althold_output);

        rpyt_out = Eigen::VectorXf(_motor_num);
        for (int i = 0;i < _motor_num;++i)
            if (_is_front_motor[i]) {
                rpyt_out(i) = 0.0f;
            }
            else {
                rpyt_out(i) = althold_output(i);
            }
    }
    else {
        Eigen::VectorXf althold_input = (Eigen::VectorXf(4) << rpyt_in(0), rpyt_in(1),
            rpyt_in(2), 0.0f).finished();

        Eigen::VectorXf althold_output;

        _althold_controller.output(althold_input, althold_output);

        rpyt_out = Eigen::VectorXf(_motor_num);
        for (int i = 0;i < _motor_num;++i)
            if (_is_front_motor[i]) {
                rpyt_out(i) = rpyt_in(3) * k_param_rotor_max_thrust;
            }
            else {
                rpyt_out(i) = althold_output(i);
            }
    }
}

void PIDQuadPlaneController::add_motor(float roll_fac, float pitch_fac, float yaw_fac,
    float throttle_fac, bool is_front) {
    
    _althold_controller.add_motor(roll_fac, pitch_fac, yaw_fac, throttle_fac);
    _is_front_motor.push_back(is_front);
    ++_motor_num;
}

void PIDQuadPlaneController::add_motors(const std::vector<Rotor> &rotors) {
    float roll_fac, pitch_fac, yaw_fac, throttle_fac;
    bool is_front;

    for (const auto &rotor : rotors) {
        auto &&rotor_pos = rotor.position_body_frame();
        auto &&rotor_dir = rotor.direction_body_frame();
        bool rotor_ccw = rotor.is_ccw();

        // Beichen Li: this is a very weak judgment and should possibly be changed in the future
        is_front = rotor_dir(0) > 0.2;

        roll_fac = is_front ? 0.0 : rotor_pos(1) > 0.2 ? -0.5 : 0.5;
        pitch_fac = is_front ? 0.0 : rotor_pos(0) > 0.2 ? 0.5 : -0.5;
        yaw_fac = is_front ? 0.0 : rotor_ccw ? 0.5 : -0.5;
        throttle_fac = is_front ? 0.0 : 1.0;

        _althold_controller.add_motor(roll_fac, pitch_fac, yaw_fac, throttle_fac);
        _is_front_motor.push_back(is_front);
    }

    _motor_num += rotors.size();
}

}