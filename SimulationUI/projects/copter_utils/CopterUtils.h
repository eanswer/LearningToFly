#pragma once
#include <string>
#include <chrono>
#include <ctime>
#include <iostream>
#include "Mesh.h"
#include "RandomGenerator.h"

#ifndef PI
    #define PI 3.14159265359f
#endif

#ifndef eps
    #define eps 1e-7
#endif
  
namespace copter_utils {
    inline std::string get_parent_directory(std::string filename) {
        int i;
        for (i = (int)filename.size() - 2;i > 0 && filename[i] != '/' && filename[i] != '\\';--i);
        return filename.substr(0, i + 1);
    }

    inline float Clamp(float x, float l, float r) {
        if (x < l) x = l;
        if (x > r) x = r;
        return x;
    }

    inline float Remap(float x, float l1, float r1, float l2, float r2) {
        x = Clamp(x, l1, r1);
        return (x - l1) / (r1 - l1) * (r2 - l2) + l2;
    }

    inline float wrap2PI(float radian) {
        for (;radian < -PI;) {
            radian += 2.0 * PI;
        }
        for (;radian > PI;) {
            radian -= 2.0 * PI;
        }
        return radian;
    }

    inline float degree2radian(float degree) {
        return degree / 180.0 * PI;
    }

    inline float radian2degree(float radian) {
        return radian / PI * 180.0;
    }

    inline Eigen::Vector3f radian2degree(Eigen::Vector3f radian) {
        return radian / PI * 180.0;
    }

    inline float sqr(float x) {
        return x * x;
    }

    inline int sqr(int x) {
        return x * x;
    }

    inline Eigen::Matrix3f SkewMatrix(Eigen::Vector3f v) {
        return (Eigen::Matrix3f() << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0).finished();
    }

    inline std::string SystemTimeStamp() {
        std::time_t now = time(0);//std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());

        char buf[100] = { 0 };
        std::strftime(buf, sizeof(buf), "%Y-%m-%d-%H-%M-%S", std::localtime(&now));

        return buf;
    }

    inline std::chrono::time_point<std::chrono::system_clock> SystemTime() {
        return std::chrono::system_clock::now();
    }

    inline double TimeDuration(std::chrono::time_point<std::chrono::system_clock> start,
        std::chrono::time_point<std::chrono::system_clock> end) {
        std::chrono::duration<double> diff = end - start;
        return diff.count();
    }

    inline bool is_zero(float x) {
        return fabs(x) < eps;
    }

    inline float RandomNoise() {
        float noise;

        float t = (float)rand() / (float)RAND_MAX;
        noise = 4.0f * t - 2.0f;

        return noise;
    }
}