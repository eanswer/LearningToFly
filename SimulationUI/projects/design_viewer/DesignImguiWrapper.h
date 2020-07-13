#pragma once
#include "imgui_wrapper.h"
#include "DesignViewerCommon.h"
#include <vector>

namespace design_viewer {
    
class DesignViewer;

class DesignImguiWrapper :
    public opengl_viewer::ImGuiWrapper {
public:
    DesignImguiWrapper(DesignViewer* parent_viewer);
    ~DesignImguiWrapper();

    void Initialize();

    void SetupUi();

    void add_status_data(std::string domain, std::string data) { 
        _status_data.push_back(std::make_pair(domain, data)); 
    }

    void set_control_mode(std::string control_mode) {
        _control_mode = control_mode;
    }

    void set_command_target(std::vector<std::pair<std::string, float> > &command_target) {
        _command_target.clear();
        for (int i = 0;i < command_target.size();++i) {
            _command_target.push_back(command_target[i]);
        }
    }

    void set_current_time(const float current_time) { _current_time = current_time; }

private:
    void DisplayMainMenu();
    void DisplayStatusWindow();
    void DisplayMassProperty();
    void DisplaySensorData();
    void DisplayCommandTarget();

    DesignViewer* _parent_viewer;

    std::vector<std::pair<std::string, std::string> > _status_data;

    std::string _control_mode;
    std::vector<std::pair<std::string, float> > _command_target;

    // Additional Variables
    float _current_time;
};

}

