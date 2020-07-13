#pragma once
#include "CopterSimulationCommon.h"
#include "CopterUtils.h"

// #define use_complex_aero

namespace copter_simulation {

class AerodynamicsDatabase
{
public:
    AerodynamicsDatabase();
    ~AerodynamicsDatabase();

    /* Initializations */
    void Initialize(float angle_of_attack_0, float area, float AR,
        Eigen::Vector3f aerodynamic_center);
    void Initialize(std::vector<float> &params);

    /* Aerodynamic Functions */
    float Cl(float angle_of_attack);
    float Cd(float angle_of_attack);
    float Cl_noisy(float angle_of_attack, bool noisy);
    float Cd_noisy(float angle_of_attack, bool noisy);

    float get_lift(float angle_of_attack, float velocity);
    float get_drag(float angle_of_attack, float velocity);
    float get_lift_noisy(float angle_of_attack, float velocity, bool noisy);
    float get_drag_noisy(float angle_of_attack, float velocity, bool noisy);
    
    /* Get Functions */
    Eigen::Vector3f aerodynamic_center() { 
        // std::cerr << "aerodynamic center = " << _aerodynamic_center.transpose() << std::endl;
        return Eigen::Vector3f(0, 0, 0);
        // return _aerodynamic_center; }
    }
    float angle_of_attack_0() { return _angle_of_attack_0; }
    float area() { return _area; }
    float AR() { return _AR; }

private:
    /* constant */
    const float _air_rho = 1.225f;

    /* Variables */
    float _angle_of_attack_0;
    float _area;
    float _AR;
    float _Cl_0;
    float _Cd_0;
    float _e;

    Eigen::Vector3f _aerodynamic_center;

    /* Variables for noise generator */
    float _Cl_std = 0.4f;
    float _Cd_std = 0.4f;
    float _Cl_const_noise, _Cd_const_noise;
};

}