#pragma once
#include "CopterUtils.h"

namespace copter_simulation {

class InertialSensor {
public:
    typedef copter_utils::RandomGenerator Random;
    typedef Eigen::Matrix<float, 12, 1> Vector12f;
    typedef Eigen::Matrix<float, 9, 1> Vector9f;

    InertialSensor(const bool noisy = false, const bool simulate_delay = false, const float delay = 0.0f);
    
    void Reset();
    void Reset(const Eigen::Quaternionf& attitude,
        const Eigen::Vector3f& angular_rate,
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity,
        const Eigen::Vector3f& acceleration);

    void Update(const Eigen::Quaternionf& attitude,
        const Eigen::Vector3f& angular_rate,
        const Eigen::Vector3f& position,
        const Eigen::Vector3f& velocity,
        const Eigen::Vector3f& acceleration,
        const float time);

    /* Helper Functions */
    static const Eigen::Matrix3f RollPitchYawToRotationMatrix(const Eigen::Vector3f& rpy);
    static const Eigen::Vector3f QuaternionToRollPitchYaw(const Eigen::Quaternionf& attitude);
    static const Eigen::Vector3f EulerRateToBodyAngularRate(const Eigen::Vector3f& rpy,
        const Eigen::Vector3f& rpy_rate);
    static const Eigen::Matrix3f EulerRateToBodyAngularRateMatrix(const Eigen::Vector3f& rpy);
    static const Eigen::Matrix3f EulerRateToBodyAngularRateMatrixInverse(const Eigen::Vector3f& rpy);
    static const Eigen::Vector3f ComputeEulerRate(const Eigen::Vector3f& rpy,
        const Eigen::Vector3f& angular_rate_body);

    /* Get Functions */
    bool is_noisy() { return _noisy; }
    Eigen::Quaternionf attitude() { return _attitude; }
    Eigen::Vector3f rpy() { return _rpy; }
    float roll() { return _rpy[0]; }
    float pitch() { return _rpy[1]; }
    float yaw() { return _rpy[2]; }
    Eigen::Vector3f rpy_rate() { return _rpy_rate; }
    float roll_rate() { return _rpy_rate[0]; }
    float pitch_rate() { return _rpy_rate[1]; }
    float yaw_rate() { return _rpy_rate[2]; }
    Eigen::Vector3f position() { return _position; }
    Eigen::Vector3f velocity() { return _velocity; }
    float altitude() { return -_position[2]; }
    Eigen::Vector3f angular_rate_world() { return _angular_rate_world; }
    Eigen::Vector3f angular_rate_body() { return _angular_rate_body; }
    Eigen::Vector3f acceleration() { return _acceleration;  }
    Vector9f current_state() {
        Vector9f state;
        state.head<3>() = _rpy;
        state.segment<3>(3) = _velocity;
        state.tail<3>() = _angular_rate_body;
        // state.tail<3>() = _angular_rate_world;
        return state;
    }
    Vector9f delayed_state() {
        if (_simulate_delay) {
            float current_time = _time_his.back();
            int his_idx = _time_his.size() - 1;
            while (his_idx > 0 && _time_his[his_idx] > current_time - _delay)
                --his_idx;
            return _state_his[his_idx];
        } else{
            return _state_his[_state_his.size() - 1];
        }
    }

    // Initialize noise
    void GenerateStateNoise();

    // Add noise to sensor data (float type)
    inline float AddNoise(float data, float mean, float std) {
        return _noisy ? data + Random::normal<float>(mean, std) : data;
    }

    // Add noise to sensor data (Eigen::Vector3f type)
    inline Eigen::Vector3f AddNoise(Eigen::Vector3f data, Eigen::Vector3f mean, Eigen::Vector3f std) {
        Eigen::Vector3f noise;
        for (int i = 0; i < 3; ++i)
            noise(i) = Random::normal<float>(mean(i), std(i));
        return _noisy ? data + noise : data;
    }

private:
    /* Gyro */
    Eigen::Quaternionf _attitude;
    Eigen::Vector3f _rpy;
    Eigen::Vector3f _angular_rate_world, _angular_rate_body;
    Eigen::Vector3f _rpy_rate;

    /* Accelerometer */
    Eigen::Vector3f _position;
    Eigen::Vector3f _velocity;
    Eigen::Vector3f _acceleration;

    /* flags */
    const bool _noisy;

    // delay
    const bool _simulate_delay;
    const float _delay;
    std::vector<Vector9f> _state_his;
    std::vector<float> _time_his;

    // last noise reset time
    float _last_noise_reset_time;

    // Noise
    Vector12f n_mean = Vector12f::Zero();
    Vector12f n_mean_range = (Vector12f() <<
        0.05f, 0.05f, 0.1f, 0.05f, 0.05f, 0.05f,
        0.1f, 0.1f, 0.1f, 0.1f, 0.1f, 0.1f
    ).finished();
    Vector12f n_std = (Vector12f() <<
        0.f, 0.f, 0.f, 0.03f, 0.03f, 0.03f,
        0.05f, 0.05f, 0.05f, 0.03f, 0.03f, 0.03f
    ).finished();
    // Vector12f n_mean_range = (Vector12f() <<
    //     0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.1f,
    //     0.02f, 0.02f, 0.02f, 0.0f, 0.0f, 0.0f
    // ).finished();
    // Vector12f n_mean_range = (Vector12f() <<
    //     0.0f, 0.0f, 0.0f, 0.02f, 0.02f, 0.1f,
    //     // 0.0f, 0.0f, 0.0f, 0.05f, 0.05f, 0.2f,
    //     0.2f, 0.2f, 0.2f, 0.0f, 0.0f, 0.0f
    //     // 1.0f, 1.0f, 0.5f, 0.0f, 0.0f, 0.0f
    // ).finished();
    // Vector12f n_std = (Vector12f() <<
    //     0.f, 0.f, 0.f, 0.005f, 0.005f, 0.005f,
    //     0.005f, 0.005f, 0.005f, 0.0005f, 0.0005f, 0.0005f
    // ).finished();
};

}