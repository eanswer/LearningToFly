#include "LightAnimator.h"
#include "DesignViewer.h"

namespace design_viewer {

LightAnimator::LightAnimator()
    : Animator(), _initial_position(0.0f, 0.0f, 0.0f), _parent_viewer(NULL) {}

void LightAnimator::Initialize(const Eigen::Vector3f& initial_position,
    DesignViewer* parent_viewer) {
    _initial_position = initial_position;
    _parent_viewer = parent_viewer;
}

const Eigen::Matrix4f LightAnimator::AnimatedModelMatrix(const float t) {
    Eigen::Vector3f copter_center =
        _parent_viewer->copter().rigid_body().position().cast<float>();
    return opengl_viewer::Translate(_initial_position + copter_center);
}

}