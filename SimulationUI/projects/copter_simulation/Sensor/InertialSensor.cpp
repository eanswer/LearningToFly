#include "InertialSensor.h"

namespace copter_simulation {

InertialSensor::InertialSensor(const bool noisy, const bool simulate_delay, const float delay)
    :_noisy(noisy), _simulate_delay(simulate_delay), _delay(delay) {
    Reset();
}

void InertialSensor::Reset() {
    Eigen::Vector3f zero;
    zero.setZero();
    Reset(Eigen::Quaternionf::Identity(), zero, zero, zero, zero);
}

void InertialSensor::GenerateStateNoise() {
    // Generate random numbers
    for (int i = 0; i < 12; ++i) {
        float range = n_mean_range(i);
        n_mean(i) = Random::uniform<float>(-range, range);
    }
}

void InertialSensor::Reset(const Eigen::Quaternionf& attitude,
    const Eigen::Vector3f& angular_rate,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity,
    const Eigen::Vector3f& acceleration) {

    GenerateStateNoise();

    _attitude = attitude;
    _angular_rate_world = AddNoise(angular_rate, n_mean.tail<3>(), n_std.tail<3>());
    _angular_rate_body = _attitude.matrix().transpose() * _angular_rate_world;
    _rpy = AddNoise(QuaternionToRollPitchYaw(attitude), n_mean.segment<3>(3), n_std.segment<3>(3));
    _rpy_rate = ComputeEulerRate(_rpy, _angular_rate_body);
    _position = AddNoise(position, n_mean.head<3>(), n_std.head<3>());
    _velocity = AddNoise(velocity, n_mean.segment<3>(6), n_std.segment<3>(6));
    _acceleration = acceleration;

    // Reset delay table
    _state_his.clear();
    _state_his.reserve(1 << 16);
    _state_his.emplace_back(current_state());
    _time_his.clear();
    _time_his.reserve(1 << 16);
    _time_his.push_back(0.0f);
}

void InertialSensor::Update(const Eigen::Quaternionf& attitude,
    const Eigen::Vector3f& angular_rate,
    const Eigen::Vector3f& position,
    const Eigen::Vector3f& velocity,
    const Eigen::Vector3f& acceleration,
    const float current_time) {
    
    _attitude = attitude;
    _angular_rate_world = AddNoise(angular_rate, n_mean.tail<3>(), n_std.tail<3>());
    _angular_rate_body = _attitude.matrix().transpose() * _angular_rate_world;
    _rpy = AddNoise(QuaternionToRollPitchYaw(attitude), n_mean.segment<3>(3), n_std.segment<3>(3));
    _rpy_rate = ComputeEulerRate(_rpy, _angular_rate_body);
    _position = AddNoise(position, n_mean.head<3>(), n_std.head<3>());
    _velocity = AddNoise(velocity, n_mean.segment<3>(6), n_std.segment<3>(6));
    _acceleration = acceleration;

    // Add record to delay table
    _state_his.emplace_back(current_state());
    _time_his.push_back(current_time);

    if (current_time - _last_noise_reset_time > 2.0) {
        // GenerateStateNoise();
        _last_noise_reset_time = current_time;
    }
}

const Eigen::Matrix3f InertialSensor::RollPitchYawToRotationMatrix(const Eigen::Vector3f& rpy) {
    const double roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
    return Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()).matrix();
}

const Eigen::Vector3f InertialSensor::QuaternionToRollPitchYaw(
    const Eigen::Quaternionf& attitude) {
    // The three angles are computed based on the slides here:
    // http://www.princeton.edu/~stengel/MAE331Lecture9.pdf
    // Page 3.
    float roll, pitch, yaw;
    const Eigen::Matrix3f R = attitude.matrix();
    const Eigen::Vector3f XI = Eigen::Vector3f::UnitX();
    const Eigen::Vector3f YI = Eigen::Vector3f::UnitY();
    const Eigen::Vector3f ZI = Eigen::Vector3f::UnitZ();
    const Eigen::Vector3f XB = R.col(0), YB = R.col(1), ZB = R.col(2);
    // Let's first rotate along XB to compute Y2.
    Eigen::Vector3f Y2 = ZI.cross(XB);
    Y2.normalize();
    float cosRoll = Y2.dot(YB);
    // Clamp cosRoll.
    if (cosRoll > 1.0) cosRoll = 1.0;
    if (cosRoll < -1.0) cosRoll = -1.0;
    roll = acos(cosRoll);
    // Check to see whether we need to swap the sign of roll.
    if (Y2.dot(ZB) > 0.0) roll = -roll;

    // Next let's rotate along Y2 so that X1 falls into XOY plane.
    Eigen::Vector3f X1 = Y2.cross(ZI);
    X1.normalize();
    float cosPitch = X1.dot(XB);
    // Clamp cosPitch.
    if (cosPitch > 1.0) cosPitch = 1.0;
    if (cosPitch < -1.0) cosPitch = -1.0;
    pitch = acos(cosPitch);
    // Check to see whether we need to swap the sign of pitch.
    if (XB[2] > 0.0) pitch = -pitch;

    // We finally need to rotate along ZI to compute yaw.
    float cosYaw = X1.dot(XI);
    // Clamp cosYaw.
    if (cosYaw > 1.0) cosYaw = 1.0;
    if (cosYaw < -1.0) cosYaw = -1.0;
    yaw = acos(cosYaw);
    if (X1[1] < 0) yaw = -yaw;
    
    return Eigen::Vector3f(roll, pitch, yaw);
}

const Eigen::Vector3f InertialSensor::EulerRateToBodyAngularRate(
    const Eigen::Vector3f& rpy,
    const Eigen::Vector3f& rpy_rate) {
    return EulerRateToBodyAngularRateMatrix(rpy) * rpy_rate;
}

const Eigen::Matrix3f InertialSensor::EulerRateToBodyAngularRateMatrix(const Eigen::Vector3f& rpy) {
    const float roll = rpy(0), pitch = rpy(1);
    const float s_roll = sin(roll), c_roll = cos(roll), s_pitch = sin(pitch), c_pitch = cos(pitch);
    return (Eigen::Matrix3f() << 1, 0, -s_pitch,
        0, c_roll, s_roll * c_pitch,
        0, -s_roll, c_roll * c_pitch).finished();
}

const Eigen::Matrix3f InertialSensor::EulerRateToBodyAngularRateMatrixInverse(const Eigen::Vector3f& rpy) {
    // Reference:
    // http://www.princeton.edu/~stengel/MAE331Lecture9.pdf.
    const float roll = rpy(0), pitch = rpy(1), yaw = rpy(2);
    const float s_roll = sin(roll), c_roll = cos(roll),
        s_pitch = sin(pitch), c_pitch = cos(pitch), t_pitch = tan(pitch);
    return (Eigen::Matrix3f() << 1, s_roll * t_pitch, c_roll * t_pitch,
        0, c_roll, -s_roll,
        0, s_roll / c_pitch, c_roll / c_pitch).finished();
}

const Eigen::Vector3f InertialSensor::ComputeEulerRate(const Eigen::Vector3f& rpy,
    const Eigen::Vector3f& angular_rate_body) {
    return EulerRateToBodyAngularRateMatrixInverse(rpy) * angular_rate_body;
}

}