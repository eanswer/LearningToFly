#include "DesignViewerTimer.h"
#include "DesignViewer.h"

namespace design_viewer {

DesignViewerTimer::DesignViewerTimer()
    : _fps(25), _dt(0.04f), _current_frame(0), _parent_viewer(nullptr) {}

DesignViewerTimer::~DesignViewerTimer() {}

void DesignViewerTimer::Initialize(const int fps, DesignViewer* parent_viewer) {
    _fps = fps;
    _dt = 1.0f / _fps;
    _current_frame = 0;
    _parent_viewer = parent_viewer;
}

const float DesignViewerTimer::CurrentTime() {

    _parent_viewer->UpdateTransmitterInput();

    const float current_time = _current_frame * _dt;
    if (!_parent_viewer->paused()) {
        ++_current_frame;
        _parent_viewer->Advance(_dt);
    }

    return current_time;
}

}