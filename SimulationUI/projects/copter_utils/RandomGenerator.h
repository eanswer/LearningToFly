#pragma once

#include <random>

namespace copter_utils {

class RandomGenerator {
public:
    template <typename RealType>
    static RealType uniform(RealType low, RealType high) {
        return std::uniform_real_distribution<RealType>(low, high)(e2);
    }

    template <typename RealType>
    static RealType normal(RealType mean, RealType stddev) {
        return std::normal_distribution<RealType>(mean, stddev)(e2);
    }

private:
    static std::random_device rd;
    static std::mt19937 e2;
};

}
