#include "ArrowAnimator.h"
#include "DesignViewer.h"

namespace design_viewer {

ArrowAnimator::ArrowAnimator()
  : Animator(), _position_body_frame(0.0f, 0.0f, 0.0f), 
  _parent_viewer(NULL) {}

void ArrowAnimator::Initialize(Eigen::Vector3f position_body_frame,
    Eigen::Vector3f direction_body_frame,
    DesignViewer* parent_viewer) { 
    _length = 0.0f;
    _position_body_frame = position_body_frame;
    _direction_body_frame = direction_body_frame;
    _parent_viewer = parent_viewer;
}

const Eigen::Matrix4f ArrowAnimator::AnimatedModelMatrix(
    const float t) {
    
    const copter_simulation::HybridCopter& copter = _parent_viewer->copter();
    
    Eigen::Matrix4f world_trans = copter.TransformBodyToWorld(0).cast<float>();

    Eigen::Matrix4f body_trans = Eigen::Matrix4f::Identity();
    Eigen::Quaternionf quat;
    quat.setFromTwoVectors(Eigen::Vector3f::UnitZ(), _direction_body_frame); // initial direction is (0, 0, 1)
    body_trans.topLeftCorner(3, 3) = Eigen::Matrix3f::Identity();
    body_trans(2, 2) = _length;
    body_trans.topLeftCorner(3, 3) = quat.matrix() * body_trans.topLeftCorner(3, 3);
    body_trans(0, 3) = _position_body_frame(0);
    body_trans(1, 3) = _position_body_frame(1);
    body_trans(2, 3) = _position_body_frame(2);
    
    Eigen::Matrix4f T = world_trans * body_trans;
    // T.topLeftCorner(3, 3) *= _length;
    
    return T;
}

}