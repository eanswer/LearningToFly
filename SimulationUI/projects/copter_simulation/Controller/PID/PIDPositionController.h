#pragma once
#include "CopterUtils.h"
#include "Sensor/InertialSensor.h"
#include "Controller/PID/PID.h"

namespace copter_simulation {

class HybridCopter;

#ifndef TARGET_Z_DIFF_LIM
# define TARGET_Z_DIFF_LIM 0.2f
#endif
#ifndef THROTTLE_HOVER
# define THROTTLE_HOVER 0.5f
#endif
#ifndef POS_Z_VEL_P
# define POS_Z_VEL_P 2.5f
#endif
#ifndef POS_Z_VEL_I
# define POS_Z_VEL_I 0.2f
#endif
#ifndef POS_Z_VEL_D
# define POS_Z_VEL_D 0.01f
#endif
#ifndef POS_Z_VEL_IMAX
# define POS_Z_VEL_IMAX 0.5f
#endif
#ifndef POS_Z_ACC_P
# define POS_Z_ACC_P 0.1f
#endif
#ifndef POS_Z_ACC_I
# define POS_Z_ACC_I 0.01f
#endif
#ifndef POS_Z_ACC_D
# define POS_Z_ACC_D 0.02f
#endif
#ifndef POS_Z_ACC_IMAX
# define POS_Z_ACC_IMAX 0.5f
#endif

// A Multi-Level controller for Position Control
// Actually a Z Controller
class PIDPositionController {
public:
    PIDPositionController(InertialSensor& sensor, HybridCopter& copter);

    void reset();

    // input is target climb rate
    // output is a throttle output in [0,1]
    void output(float target_climb_rate, float& throttle_output);

private:
    void set_alt_target_from_climb_rate(float climb_rate);
    // control the altitude
    void z_control(float& throttle_output);
    // control the altitude rate
    void z_rate_control(float target_climb_rate, float& throttle_output);
    // control the altitude acceleration
    void z_acc_control(float target_climb_acc, float& throttle_output);

    InertialSensor& _sensor;
    HybridCopter& _copter;

    /* Variables for controller */
    float _target_z;                // z here represents altitude
    float _target_z_vel;
    float _target_z_acc;
    float _dt;
    float _throttle_hover;          // estimated throttle for hovering, [0, 1]

    /* PID Controllers */
    PID _pid_z_vel;
    PID _pid_z_acc;
};

}