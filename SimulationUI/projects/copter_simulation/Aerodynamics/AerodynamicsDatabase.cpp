#include "Aerodynamics/AerodynamicsDatabase.h"

namespace copter_simulation {

AerodynamicsDatabase::AerodynamicsDatabase()
    :_angle_of_attack_0(PI / 18.0f), _area(0.0f),
    _AR(0.0f), _Cl_0(0.2f), _Cd_0(0.0f), _e(0.9f) {}


AerodynamicsDatabase::~AerodynamicsDatabase() {}

void AerodynamicsDatabase::Initialize(float angle_of_attack_0, float area,
    float AR, Eigen::Vector3f aerodynamic_center) {
    _angle_of_attack_0 = angle_of_attack_0;
    _area = area;
    _AR = AR;
    _aerodynamic_center = aerodynamic_center;
}

void AerodynamicsDatabase::Initialize(std::vector<float> &params) {
}

float AerodynamicsDatabase::Cl(float angle_of_attack) {
    float aoa = angle_of_attack + _angle_of_attack_0;
    return 2.0 * std::sin(aoa) * cos(aoa);
}

float AerodynamicsDatabase::Cd(float angle_of_attack) {
    float aoa = angle_of_attack + _angle_of_attack_0;
    return 2.0 * copter_utils::sqr(std::sin(aoa)) + 0.1;
}

float AerodynamicsDatabase::get_lift(float angle_of_attack, float velocity) {
    float lift_force = Cl(angle_of_attack) * _air_rho * copter_utils::sqr(velocity) * _area / 2.0f;

    return lift_force;
}

float AerodynamicsDatabase::get_drag(float angle_of_attack, float velocity) {
    double drag_force = Cd(angle_of_attack) * _air_rho * copter_utils::sqr(velocity) * _area / 2.0f;

    return drag_force;
}

}
