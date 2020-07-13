#include "DesignViewerKeyboardHandler.h"
#include "DesignViewer.h"

namespace design_viewer {

DesignViewerKeyboardHandler::DesignViewerKeyboardHandler()
    : _parent_viewer(nullptr) {}

DesignViewerKeyboardHandler::~DesignViewerKeyboardHandler() {}

void DesignViewerKeyboardHandler::Initialize(const bool paused,
    DesignViewer* parent_viewer) {
    
    _parent_viewer = parent_viewer;
    _parent_viewer->set_paused(paused);
}

void DesignViewerKeyboardHandler::KeyCallback(const int key, const int action) {
    
    // Pause.
    if (_parent_viewer->focus_on_simulation()) {
        if (key == GLFW_KEY_P && action == GLFW_PRESS) _parent_viewer->flip_paused();

        if (action == GLFW_PRESS) {
            switch (key) {
            case GLFW_KEY_UP:
                _parent_viewer->set_up_pressed(true);
                break;
            case GLFW_KEY_DOWN:
                _parent_viewer->set_down_pressed(true);
                break;
            case GLFW_KEY_LEFT:
                _parent_viewer->set_left_pressed(true);
                break;
            case GLFW_KEY_RIGHT:
                _parent_viewer->set_right_pressed(true);
                break;
            case GLFW_KEY_A:
                _parent_viewer->set_a_pressed(true);
                break;
            case GLFW_KEY_D:
                _parent_viewer->set_d_pressed(true);
                break;
            case GLFW_KEY_W:
                _parent_viewer->set_w_pressed(true);
                break;
            case GLFW_KEY_S:
                _parent_viewer->set_s_pressed(true);
                break;
            case GLFW_KEY_E:
                _parent_viewer->set_e_pressed(true);
                break;
            case GLFW_KEY_R:
                _parent_viewer->set_r_pressed(true);
                break;
            case GLFW_KEY_C:
                _parent_viewer->switch_controller();
                break;
            case GLFW_KEY_1:
                _parent_viewer->set_mode_switch(0);
                break;
            case GLFW_KEY_2:
                _parent_viewer->set_mode_switch(1);
                break;
            default:
                break;
            }
        }
        else if (action == GLFW_RELEASE) {
            switch (key) {
            case GLFW_KEY_UP:
                _parent_viewer->set_up_pressed(false);
                break;
            case GLFW_KEY_DOWN:
                _parent_viewer->set_down_pressed(false);
                break;
            case GLFW_KEY_LEFT:
                _parent_viewer->set_left_pressed(false);
                break;
            case GLFW_KEY_RIGHT:
                _parent_viewer->set_right_pressed(false);
                break;
            case GLFW_KEY_A:
                _parent_viewer->set_a_pressed(false);
                break;
            case GLFW_KEY_D:
                _parent_viewer->set_d_pressed(false);
                break;
            case GLFW_KEY_W:
                _parent_viewer->set_w_pressed(false);
                break;
            case GLFW_KEY_S:
                _parent_viewer->set_s_pressed(false);
                break;
            case GLFW_KEY_E:
                _parent_viewer->set_e_pressed(false);
                break;
            case GLFW_KEY_R:
                _parent_viewer->set_r_pressed(false);
                break;
            default:
                break;
            }
        }
    }
}

}
