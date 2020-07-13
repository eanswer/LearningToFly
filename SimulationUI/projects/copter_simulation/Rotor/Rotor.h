#pragma once
#include "CopterSimulationCommon.h"

namespace copter_simulation {

class Rotor
{
public:
    Rotor(const Eigen::Vector3f& direction,
        const Eigen::Vector3f& position, const bool is_ccw,
        float motor_mass);
    ~Rotor();

    // the API that controller send the desired_thrust to each rotor
    void output(const float desired_thrust);

    // output the thrust/torque in body frame to simulation
    Eigen::Vector3f get_thrust();
    Eigen::Vector3f get_torque();

    Eigen::Vector3f position_body_frame() const { return _position_body_frame; }
    Eigen::Vector3f direction_body_frame() const { return _direction_body_frame; }
    bool            is_ccw() const { return _is_ccw; }
    float           mass() const { return _mass; }
    float           desired_thrust() const { return _desired_thrust; }

private:
    Eigen::Vector3f _position_body_frame;
    Eigen::Vector3f _direction_body_frame;
    bool _is_ccw;
    float _desired_thrust;
    float _mass;
};

}

