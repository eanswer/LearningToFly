#pragma once

#include "animator.h"

namespace design_viewer {

class DesignViewer;

class CopterRotorAnimator : public opengl_viewer::Animator {
public:
    CopterRotorAnimator();

    void Initialize(const int rotor_id, DesignViewer* parent_viewer, bool rotate = false);

    const Eigen::Matrix4f AnimatedModelMatrix(const float t);

private:
    int _id;
    DesignViewer* _parent_viewer;
    bool _rotate;
    float _angle, _last_time;
};

}