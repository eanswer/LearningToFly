#pragma once
#include "CopterUtils.h"
#include "Sensor/InertialSensor.h"
#include "Controller/PID/PID.h"

namespace copter_simulation {

class HybridCopter;

// default rate controller PID gains
#ifndef RATE_RP_P
# define RATE_RP_P           0.135f
#endif
#ifndef RATE_RP_I
# define RATE_RP_I           0.090f
#endif
#ifndef RATE_RP_D
# define RATE_RP_D           0.0036f
#endif
#ifndef RATE_RP_IMAX
# define RATE_RP_IMAX         0.5f
#endif
#ifndef RATE_RP_FILT_HZ
# define RATE_RP_FILT_HZ      20.0f
#endif
#ifndef RATE_YAW_P
# define RATE_YAW_P           1.0f//0.7f
#endif
#ifndef RATE_YAW_I
# define RATE_YAW_I           0.018f
#endif
#ifndef RATE_YAW_D
# define RATE_YAW_D           0.0f
#endif
#ifndef RATE_YAW_IMAX
# define RATE_YAW_IMAX        0.5f
#endif
#ifndef RATE_YAW_FILT_HZ
# define RATE_YAW_FILT_HZ     2.5f
#endif

// A Low-Level controller for Attitude Rate Control
class PIDAttitudeRateController{
public:
    PIDAttitudeRateController(InertialSensor& sensor, HybridCopter& copter);

    void reset();

    // input is Vector3f (target_roll_rate, target_pitch_rate, target_yaw_rate)
    // output is Vector3f (roll_output, pitch_output, yaw_output) in [-1, 1]
    void output(const Eigen::Vector3f& target_euler_rate_roll_pitch_yaw, 
        Eigen::Vector3f& roll_pitch_yaw_output);

private:
    float compute_pid(PID& pid, float actual, float target);

    InertialSensor& _sensor;
    HybridCopter& _copter;

    /* Low-Level controllers */
    PID _roll_rate_pid, _pitch_rate_pid, _yaw_rate_pid;
};

}