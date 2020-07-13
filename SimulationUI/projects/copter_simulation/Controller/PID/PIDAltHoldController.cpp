#include "Controller/PID/PIDAltHoldController.h"
#include "HybridCopter.h"

namespace copter_simulation {

    PIDAltHoldController::PIDAltHoldController(InertialSensor& sensor,
        HybridCopter& copter)
        : HighLevelController(sensor, copter),
        _attitude_controller(sensor, copter),
        _attitude_rate_controller(sensor, copter),
        _position_controller(sensor, copter) {
        
        _motor_num = 0;
        _roll_fac.clear();
        _pitch_fac.clear();
        _yaw_fac.clear();
        _throttle_fac.clear();
    }

    void PIDAltHoldController::reset() {
        _attitude_controller.reset();
        _attitude_rate_controller.reset();
        _position_controller.reset();
    }

    void PIDAltHoldController::output(const Eigen::VectorXf& rpyt_in,
        Eigen::VectorXf& rpyt_out) {
        
        Eigen::Vector3f target_euler_rate_roll_pitch_yaw;
        Eigen::Vector3f rpy_output;
        float throttle_output;

        _attitude_controller.output(rpyt_in.head(3), target_euler_rate_roll_pitch_yaw);
        _attitude_rate_controller.output(target_euler_rate_roll_pitch_yaw, rpy_output);
        _position_controller.output(rpyt_in(3), throttle_output);

        rpyt_out = convert_to_motor_thrust(rpy_output, throttle_output);
    }

    void PIDAltHoldController::add_motor(float roll_fac, float pitch_fac, 
        float yaw_fac, float throttle_fac) {
        ++ _motor_num;
        _roll_fac.push_back(roll_fac);
        _pitch_fac.push_back(pitch_fac);
        _yaw_fac.push_back(yaw_fac);
        _throttle_fac.push_back(throttle_fac);
    }

    void PIDAltHoldController::add_motors(const std::vector<Rotor> &rotors) {
        float roll_fac, pitch_fac, yaw_fac, throttle_fac;
        bool is_front;

        for (const auto &rotor : rotors) {
            auto &&rotor_pos = rotor.position_body_frame();
            auto &&rotor_dir = rotor.direction_body_frame();
            bool rotor_ccw = rotor.is_ccw();

            // Beichen Li: this is a very weak judgment and should possibly be changed in the future
            is_front = rotor_dir(0) > 0;

            roll_fac = is_front ? 0.0 : rotor_pos(1) > 0 ? -0.5 : 0.5;
            pitch_fac = is_front ? 0.0 : rotor_pos(0) > 0 ? 0.5 : -0.5;
            yaw_fac = is_front ? 0.0 : rotor_ccw ? 0.5 : -0.5;
            throttle_fac = is_front ? 0.0 : 1.0;

            _roll_fac.push_back(roll_fac);
            _pitch_fac.push_back(pitch_fac);
            _yaw_fac.push_back(yaw_fac);
            _throttle_fac.push_back(throttle_fac);
        }

        _motor_num += rotors.size();
    }

    Eigen::VectorXf PIDAltHoldController::convert_to_motor_thrust(Eigen::Vector3f& rpy_output,
        float throttle_output) {

        Eigen::VectorXf motor_output = Eigen::VectorXf::Zero(_copter.rotors().size());

        for (int i = 0;i < _motor_num;++i) {
            float scaled_thrust = throttle_output * _throttle_fac[i]
                + rpy_output[0] * _roll_fac[i]
                + rpy_output[1] * _pitch_fac[i]
                + rpy_output[2] * _yaw_fac[i];
            scaled_thrust = copter_utils::Clamp(scaled_thrust, 0.0f, 1.0f);
            motor_output[i] = copter_utils::Remap(scaled_thrust, 0.0f, 1.0f, 0.0f, k_param_rotor_max_thrust);
        }

        return motor_output;
    }
}