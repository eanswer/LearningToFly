#pragma once

#include "animator.h"

namespace design_viewer {

class DesignViewer;

class ArrowAnimator : public opengl_viewer::Animator {
public:
    ArrowAnimator();

    void Initialize(Eigen::Vector3f position_body_frame,
        Eigen::Vector3f direction_body_frame,
        DesignViewer* parent_viewer);

    const Eigen::Matrix4f AnimatedModelMatrix(const float t);

    void set_length(const float length) { _length = length; }
    
    void set_position_body_frame(const Eigen::Vector3f position_body_frame) {
        _position_body_frame = position_body_frame;
    }

    void set_direction_body_frame(const Eigen::Vector3f direction_body_frame) {
        _direction_body_frame = direction_body_frame;
        _direction_body_frame.normalize();
    }

private:
    DesignViewer* _parent_viewer;

    /* Variables */
    float _length;    // should be normalized by max thrust
    Eigen::Vector3f _position_body_frame;
    Eigen::Vector3f _direction_body_frame; 
};

}