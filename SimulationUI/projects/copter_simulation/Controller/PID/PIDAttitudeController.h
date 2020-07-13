#pragma once
#include "CopterUtils.h"
#include "Sensor/InertialSensor.h"
#include "Controller/PID/PID.h"

namespace copter_simulation {

class HybridCopter;

// default rate controller PID gains
#ifndef RP_P
# define RP_P           1.5f
#endif
#ifndef RP_I
# define RP_I           0.900f
#endif
#ifndef RP_D
# define RP_D           0.036f
#endif
#ifndef RP_IMAX
# define RP_IMAX         1.5f
#endif
#ifndef RP_FILT_HZ
# define RP_FILT_HZ      20.0f
#endif

// A 2nd Level controller for Attitude Control
// Simple version: control rpy, TODO: control quaternion
class PIDAttitudeController {
public:
    PIDAttitudeController(InertialSensor& sensor, HybridCopter& copter);

    void reset();

    // input is Vector3f (target_roll, target_pitch, target_yaw_rate)
    // output is Vector3f (target_roll_rate, target_pitch_rate, target_yaw_rate)
    void output(const Eigen::Vector3f& target_euler_roll_pitch_euler_rate_yaw, 
        Eigen::Vector3f& target_euler_rate_roll_pitch_yaw);

private:
    float compute_pid(PID& pid, float actual, float target);

    InertialSensor& _sensor;
    HybridCopter& _copter;

    /* Low-Level controllers */
    PID _roll_pid, _pitch_pid;
};

}