// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/// @file   AP_MotorsHybrid.h
/// @brief  Motor control class for frames used for HybridUAV

#ifndef __AP_MOTORS_HYBRID_H__
#define __AP_MOTORS_HYBRID_H__

#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <RC_Channel/RC_Channel.h>     // RC Channel Library
#include "AP_MotorsMatrix.h"    // Parent Motors Matrix library

// Forward declaration.
class Copter;

#define RUN_PID     1
#define RUN_NN      1

/// @class      AP_MotorsHybrid
class AP_MotorsHybrid : public AP_MotorsMatrix {
public:
    /// Constructor
    AP_MotorsHybrid(uint16_t loop_rate, uint16_t speed_hz, Copter& cop)
        : AP_MotorsMatrix(loop_rate, speed_hz), _copter(cop), I_dt(0.0025) { }
    virtual ~AP_MotorsHybrid() {}

    void                set_radios_switch(uint16_t switch_CH5, uint16_t switch_CH6);

    // setup_motors - configures the motors for rotors.
    virtual void        setup_motors(motor_frame_class frame_class, motor_frame_type frame_type);
    void                output_to_motors();

protected:
    // acquire observation vector
    void                get_observation_vector(float ob[]);
    void                get_state(float state[]);
    void                collect_rpy();
    void                get_velocity_body(float vel[]);
    void                get_angular_velocity(float omega[]);

    // run neural network controller
    void                pi_act(float ob[], float action[]);

    // output - sends commands to the motors
    void                output_armed_stabilizing();
    void                thrust_compensation(void) override;

private:
    // Points to the Copter class so that we can get all kinds of sensor's data.
    Copter&       _copter;

    // mode
    uint16_t policy_mode; // 0 for our own policy, 1 for althold mode
    uint16_t flight_mode; // 0 for copter mode, 1 for gliding mode
    bool     urgent_shut_down;
    
    // target
    float target_vx, target_vy, target_vz;
    float target_yaw_diff;

    // I term
    float I_error[4];
    float I_dt;
    bool I_activated;
    
    // state
    float roll, pitch, yaw;
    float last_omega[3];
    
    // initial values
    float yaw_0;

    bool initialization_finished;
    double initial_yaw_sum;
    int yaw_count;

    float _thrust_rpyt_out_NN[AP_MOTORS_MAX_NUM_MOTORS];

    // NN motor output
    int16_t _motor_out_NN[AP_MOTORS_MAX_NUM_MOTORS];
};

#endif  // AP_MOTORSHybrid
