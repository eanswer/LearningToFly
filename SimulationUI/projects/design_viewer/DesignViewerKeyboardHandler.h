#pragma once
#include "keyboard_handler.h"

namespace design_viewer {

class DesignViewer;

class DesignViewerKeyboardHandler :
    public opengl_viewer::KeyboardHandler
{
public:
    DesignViewerKeyboardHandler();
    ~DesignViewerKeyboardHandler();

    void Initialize(const bool paused, DesignViewer* parent_viewer);

    void KeyCallback(const int key, const int action);

private:

    DesignViewer* _parent_viewer;
};

}