#pragma once
#include "Controller/HighLevelController.h"
#include "Controller/PID/PIDAltHoldController.h"
#include "CopterSimulationCommon.h"
#include "Rotor/Rotor.h"

namespace copter_simulation {

class PIDQuadPlaneController : public HighLevelController {
public:
    PIDQuadPlaneController(InertialSensor& sensor, HybridCopter& copter);
    
    void reset();

    // input is target roll, target pitch, target yaw rate, target front throttle
    void output(const Eigen::VectorXf& rpyt_in, Eigen::VectorXf& rpyt_out);

    void add_motor(float roll_fac, float pitch_fac, float yaw_fac,
        float throttle_fac, bool is_front);
    
    void add_motors(const std::vector<Rotor> &rotors);

    void set_mode(FlightMode mode) { _mode = mode; }

private:
    /* sub controllers */
    PIDAltHoldController _althold_controller;

    /* Variables */
    FlightMode _mode;
    std::vector<bool> _is_front_motor;
    int _motor_num;
};

}