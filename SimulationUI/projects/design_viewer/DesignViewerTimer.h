#pragma once
#include "timer.h"

namespace design_viewer {

class DesignViewer;

class DesignViewerTimer :
    public opengl_viewer::Timer
{
public:
    DesignViewerTimer();
    ~DesignViewerTimer();

    void Initialize(const int fps, DesignViewer* parent_viewer);

    const float CurrentTime();

private:
    int _fps;
    float _dt;
    float _current_frame;
    DesignViewer* _parent_viewer;
};

}