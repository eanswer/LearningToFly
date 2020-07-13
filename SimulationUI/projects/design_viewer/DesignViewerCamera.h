#pragma once
#include "Camera.h"

namespace design_viewer {

class DesignViewer;

class DesignViewerCamera : public opengl_viewer::Camera
{
public:
    DesignViewerCamera();
    ~DesignViewerCamera();

    void Initialize(Eigen::Vector3f initial_camera_pos, 
        Eigen::Vector3f initial_lookat, DesignViewer* parent_viewer);

    const Eigen::Vector3f CameraPos();
    const Eigen::Vector3f LookAt();

private:
    Eigen::Vector3f _initial_camera_pos;
    Eigen::Vector3f _initial_lookat;
    DesignViewer* _parent_viewer;
};

}

