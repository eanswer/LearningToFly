#pragma once

#include "animator.h"

namespace design_viewer {

class DesignViewer;

class LightAnimator : public opengl_viewer::Animator {
public:
    LightAnimator();

    void Initialize(const Eigen::Vector3f& initial_position,
        DesignViewer* parent_viewer);

    const Eigen::Matrix4f AnimatedModelMatrix(const float t);

private:
    Eigen::Vector3f _initial_position;
    DesignViewer* _parent_viewer;
};

}