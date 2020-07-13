#pragma once

namespace copter_simulation {

class PID {
public:
    PID(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, 
        float imax = 0.0f, float filt_hz = 1.0f, float dt = 1.0f);

    void reset();
    void reset_I();
    void reset_filter();

    /* input */
    void set_input_filter_all(float input);
    void set_input_filter_d(float input);

    /* get control output */
    float get_pid();
    float get_pi();
    float get_p();
    float get_i();
    float get_d();
    
    /* get functions */
    float imax() const { return _imax; }
    float filt_alpha();
    float kp() const { return _kp; }
    float ki() const { return _ki; }
    float kd() const { return _kd; }

private:
    /* parameters */
    float _kp, _ki, _kd;
    float _imax;
    float _filt_hz, _filt_alpha;
    float _dt;

    /* computed variables */
    float _integral;
    float _input;
    float _derivative;

    /* flags */
    bool _reset_filter, _reset_filt_alpha;
};

}