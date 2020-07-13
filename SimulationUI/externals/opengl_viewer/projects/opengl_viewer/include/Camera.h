#pragma once
#include <Eigen/Dense>

namespace opengl_viewer {

class Camera {
public:
    virtual ~Camera() {}

    virtual const Eigen::Vector3f CameraPos() { return Eigen::Vector3f::Zero(); }
    virtual const Eigen::Vector3f LookAt() { return Eigen::Vector3f::Zero(); }
};

}