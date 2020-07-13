#pragma once
#include "Sensor/InertialSensor.h"

namespace copter_simulation {

class HybridCopter;

class HighLevelController {
public:
    HighLevelController(InertialSensor& sensor, HybridCopter& copter)
        : _sensor(sensor), _copter(copter) {}
    
    ~HighLevelController() {}

    virtual void output(const Eigen::VectorXf& input,
        Eigen::VectorXf& output) = 0;

protected:
    InertialSensor& _sensor;
    HybridCopter& _copter;
};

}