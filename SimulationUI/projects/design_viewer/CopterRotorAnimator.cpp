#include "CopterRotorAnimator.h"
#include "DesignViewer.h"

namespace design_viewer {

CopterRotorAnimator::CopterRotorAnimator()
  : Animator(), _id(-1), _parent_viewer(NULL), _rotate(false), _angle(0.0f), _last_time(0.0f) {}

void CopterRotorAnimator::Initialize(const int rotor_id,
    DesignViewer* parent_viewer, bool rotate) { 
    
    _id = rotor_id;
    _parent_viewer = parent_viewer;
    _rotate = rotate;
    _angle = _last_time = 0.0f;
}

const Eigen::Matrix4f CopterRotorAnimator::AnimatedModelMatrix(
    const float t) {
    
    const copter_simulation::HybridCopter& copter = _parent_viewer->copter();
    const copter_simulation::Rotor& rotor = copter.rotors()[_id];

    const float base_speed = M_PI * 10;
    _angle += (base_speed + 5 * copter.last_action(_id)) * (t - _last_time);
    _last_time = t;

    Eigen::Matrix4f rot_matrix;
    rot_matrix.setIdentity();
    if (_rotate)
        rot_matrix.topLeftCorner(3, 3) = Eigen::AngleAxisf(rotor.is_ccw() ? _angle : -_angle, Eigen::Vector3f::UnitZ()).toRotationMatrix();

    Eigen::Matrix4f trans_matrix;
    trans_matrix.setIdentity();
    trans_matrix.col(3).head<3>() = rotor.position_body_frame();
    trans_matrix.topLeftCorner(3, 3) = Eigen::Quaternionf::FromTwoVectors(Eigen::Vector3f::UnitZ(), rotor.direction_body_frame()).toRotationMatrix();

    return copter.TransformBodyToWorld(_id) * trans_matrix * rot_matrix;
}

}