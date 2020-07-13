#include "Rotor/Rotor.h"

namespace copter_simulation {

Rotor::Rotor(const Eigen::Vector3f& direction,
    const Eigen::Vector3f& position, const bool is_ccw,
    float motor_mass)
    : _direction_body_frame(direction), _position_body_frame(position),
    _is_ccw(is_ccw), _mass(motor_mass) {}


Rotor::~Rotor()
{
}

void Rotor::output(const float desired_thrust) {
    _desired_thrust = std::min(desired_thrust, k_param_rotor_max_thrust);
}

Eigen::Vector3f Rotor::get_thrust() {
    return _direction_body_frame * _desired_thrust;
}

Eigen::Vector3f Rotor::get_torque() {
    return (_is_ccw ? -1.0 : 1.0) * k_param_torque_to_thrust_ratio * get_thrust();
}

}