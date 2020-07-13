#pragma once

#include "animator.h"

namespace design_viewer {

class DesignViewer;

class CopterAnimator : public opengl_viewer::Animator {
public:
    CopterAnimator();

    void Initialize(const int component_id, DesignViewer* parent_viewer);

    const Eigen::Matrix4f AnimatedModelMatrix(const float t);

private:
    int _id;
    DesignViewer* _parent_viewer;
};

}