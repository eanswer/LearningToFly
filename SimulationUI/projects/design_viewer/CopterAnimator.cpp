#include "CopterAnimator.h"
#include "DesignViewer.h"

namespace design_viewer {

CopterAnimator::CopterAnimator()
  : Animator(), _id(-1), _parent_viewer(NULL) {}

void CopterAnimator::Initialize(const int component_id,
    DesignViewer* parent_viewer) { 
    
    _id = component_id;
    _parent_viewer = parent_viewer;
}

const Eigen::Matrix4f CopterAnimator::AnimatedModelMatrix(
    const float t) {
    
    const copter_simulation::HybridCopter& copter = _parent_viewer->copter();
  
    return copter.TransformBodyToWorld(_id);
}

}