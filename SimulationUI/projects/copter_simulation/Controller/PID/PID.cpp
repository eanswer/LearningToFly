#include "Controller/PID/PID.h"
#include "CopterUtils.h"

namespace copter_simulation {

PID::PID(float kp, float ki, float kd, 
    float imax, float filt_hz, float dt)
    :_kp(kp), _ki(ki), _kd(kd), 
     _filt_hz(filt_hz), _dt(dt),
     _integral(0.0f), _input(0.0f), _derivative(0.0f) {
    
    _imax = std::fabs(imax);
    // reset input filter to first value received
    _reset_filter = true;
    _reset_filt_alpha = true;
}

void PID::reset() {
    reset_I();
    reset_filter();
}

void PID::reset_I() {
    _integral = 0.0f;
}

void PID::reset_filter() {
    _reset_filter = true;
}

// set_input_filter_all - set input to PID controller
//  input is filtered before the PID controllers are run
//  this should be called before any other calls to get_p, get_i or get_d
void PID::set_input_filter_all(float input) {
    if (!std::isfinite(input)) {
        return;
    }

    // reset input filter to value received
    if (_reset_filter) {
        _reset_filter = false;
        _input = input;
        _derivative = 0.0f;
    }

    // update filter and calculate derivative
    float input_filt_change = filt_alpha() * (input - _input);
    _input = _input + input_filt_change;
    if (_dt > 0.0f) {
        _derivative = input_filt_change / _dt;
    }
}

// set_input_filter_d - set input to PID controller
//  only input to the D portion of the controller is filtered
//  this should be called before any other calls to get_p, get_i or get_d
void PID::set_input_filter_d(float input) {
    if (!std::isfinite(input)) {
        return;
    }

    if (_reset_filter) {
        _reset_filter = false;
        _input = input;
        _derivative = 0.0f;
    }

    if (_dt > 0.0f) {
        float derivative = (input - _input) / _dt;
        _derivative = _derivative + filt_alpha() * (derivative - _derivative);
    }

    _input = input;
}

float PID::get_p() {
    return _kp * _input;
}

float PID::get_i() {
    if (!copter_utils::is_zero(_ki) && !copter_utils::is_zero(_dt)) {
        _integral += ((float)_input * _ki) * _dt;
        _integral = copter_utils::Clamp(_integral, -_imax, _imax);
        // std::cerr << "I = " << _integral << std::endl;
        return _integral;
    }
    else {
        return 0.0f;
    }
}

float PID::get_d() {
    return _kd * _derivative;
}

float PID::get_pi() {
    return get_p() + get_i();
}

float PID::get_pid() {
    return get_p() + get_i() + get_d();
}

float PID::filt_alpha() {
    if (!_reset_filt_alpha) {
        return _filt_alpha;
    }
    else {
        _reset_filt_alpha = true;
        if (copter_utils::is_zero(_filt_hz)) {
            _filt_alpha = 1.0f;
        }
        else {
            float rc = 1.0 / (PI * 2.0f * _filt_hz);
            _filt_alpha =  _dt / (_dt + rc);
        }
        return _filt_alpha;
    }
}

}