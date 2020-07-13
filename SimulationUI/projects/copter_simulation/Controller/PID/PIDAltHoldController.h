#pragma once
#include "CopterUtils.h"
#include "Sensor/InertialSensor.h"
#include "Controller/PID/PIDPositionController.h"
#include "Controller/PID/PIDAttitudeController.h"
#include "Controller/PID/PIDAttitudeRateController.h"
#include "Controller/HighLevelController.h"
#include "Rotor/Rotor.h"

namespace copter_simulation {

class PIDAltHoldController : public HighLevelController {
public:
    PIDAltHoldController(InertialSensor& sensor, HybridCopter& copter);

    void reset();

    // input is target roll, target pitch, target yaw rate, target climb rate
    void output(const Eigen::VectorXf& rpyt_in, Eigen::VectorXf& rpyt_out);

    void add_motor(float roll_fac, float pitch_fac, float yaw_fac, 
        float throttle_fac);

    void add_motors(const std::vector<Rotor> &rotors);

private:
    Eigen::VectorXf convert_to_motor_thrust(Eigen::Vector3f& rpy_output,
        float throttle_output);

    /* sub controllers */
    PIDAttitudeController _attitude_controller;
    PIDAttitudeRateController _attitude_rate_controller;
    PIDPositionController _position_controller;

    /* Motor Matrix */
    std::vector<float> _roll_fac;
    std::vector<float> _pitch_fac;
    std::vector<float> _yaw_fac;
    std::vector<float> _throttle_fac;
    int _motor_num;
};

}