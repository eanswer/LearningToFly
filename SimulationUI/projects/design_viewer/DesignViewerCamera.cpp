#include "DesignViewerCamera.h"
#include "DesignViewer.h"

namespace design_viewer {

DesignViewerCamera::DesignViewerCamera()
{
}


DesignViewerCamera::~DesignViewerCamera()
{
}

void DesignViewerCamera::Initialize(Eigen::Vector3f initial_camera_pos,
    Eigen::Vector3f initial_lookat, 
    DesignViewer* parent_viewer) {
    _initial_camera_pos = initial_camera_pos;
    _initial_lookat = initial_lookat;
    _parent_viewer = parent_viewer;
}

const Eigen::Vector3f DesignViewerCamera::CameraPos() {
    return _initial_camera_pos + _parent_viewer->copter_position();
}

const Eigen::Vector3f DesignViewerCamera::LookAt() {
    return _initial_lookat + _parent_viewer->copter_position();
}

}
