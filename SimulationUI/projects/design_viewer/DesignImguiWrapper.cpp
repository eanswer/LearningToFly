#include "DesignImguiWrapper.h"
#include "imgui.h"
#include "DesignViewer.h"
#include <iostream>

namespace design_viewer {

// public functions

DesignImguiWrapper::DesignImguiWrapper(DesignViewer* parent_viewer) 
    : _parent_viewer(parent_viewer),
    _current_time(0.0f) {}


DesignImguiWrapper::~DesignImguiWrapper() {}

void DesignImguiWrapper::Initialize() {
    _status_data.clear();
    _control_mode = "";
    _command_target.clear();
}

void DesignImguiWrapper::SetupUi() {
    ImGui::StyleColorsDark();
    
    DisplayMainMenu();

    DisplayStatusWindow();

    DisplayMassProperty();

    DisplaySensorData();

    DisplayCommandTarget();
}

// private functions

void DesignImguiWrapper::DisplayMainMenu() {
    static bool show_open_config_modal = false;
    static char config_file_name[128];

    if (ImGui::BeginMainMenuBar())
    {
        if (ImGui::BeginMenu("File"))
        {
            if (ImGui::MenuItem("Open Config")) {
                show_open_config_modal = true;
            }

            ImGui::EndMenu();
        }
        ImGui::EndMainMenuBar();
    }

    if (show_open_config_modal) {
        ImGui::OpenPopup("Open Model Config");
        ImGui::SetNextWindowPos(ImVec2(20, 20), ImGuiSetCond_Appearing);
        
    }
    
    if (ImGui::BeginPopupModal("Open Model Config", &show_open_config_modal, ImGuiWindowFlags_AlwaysAutoResize))
    {
        _parent_viewer->set_focus_on_simulation(false);

        ImGui::Text("Config File Name");
        ImGui::SameLine();
        ImGui::InputText("", config_file_name, 128);

        ImGui::Separator();

        std::string config_file_display_location = design_viewer::k_param_config_root_subdir + config_file_name;
        std::string config_file_location = design_viewer::k_param_project_source_dir + design_viewer::k_param_config_root_subdir + config_file_name;

        ImGui::Text("Full Config File Location");
        ImGui::TextColored(ImVec4(1, 0, 0, 1), config_file_display_location.c_str());
        ImGui::Separator();
        
        if (ImGui::Button("Open", ImVec2(120, 0))) { 
            bool success = _parent_viewer->copter().LoadFromConfigFile(config_file_location);
            if (success) {
                std::cerr << "open file succeed" << std::endl;
                _parent_viewer->set_focus_on_simulation(true);
                show_open_config_modal = false;
                _parent_viewer->LoadCopter();
            }
        }
        ImGui::SameLine();
        if (ImGui::Button("Cancel", ImVec2(120, 0))) { 
            show_open_config_modal = false; 
        }

        ImGui::EndPopup();
    }

}

void DesignImguiWrapper::DisplayStatusWindow() {
    ImGui::SetNextWindowSize(ImVec2(350.0f, 200.0f), ImGuiSetCond_Once);
    ImGui::SetNextWindowPos(ImVec2(20.0f, 420.0f), ImGuiSetCond_Once);
    ImGui::Begin("Status");

    ImGui::Text("> [simulation time] %.3f second", _current_time);
    for (int i = std::max(0, (int)_status_data.size() - 8) ;i < _status_data.size();++i)
        ImGui::Text("> [%s] %s", _status_data[i].first.c_str(), _status_data[i].second.c_str());

    ImGui::End();
}

void DesignImguiWrapper::DisplayMassProperty() {
    ImGui::SetNextWindowSize(ImVec2(350.0f, 110.0f), ImGuiSetCond_Once);
    ImGui::SetNextWindowPos(ImVec2(20.0f, 20.0f), ImGuiSetCond_Once);
    ImGui::Begin("Mass Property");
    
    const copter_simulation::RigidBody& copter_body = 
        _parent_viewer->copter().rigid_body();
    
    const double mass = copter_body.mass();
    ImGui::BulletText(("Mass (kg): " + std::to_string(mass)).c_str());

    const Eigen::Matrix3d inertia_body = copter_body.inertia_body();
    std::stringstream inertia_str;
    inertia_str << inertia_body;
    ImGui::BulletText(("Moment of Inertia (kgm^2): \n"
        + inertia_str.str()).c_str());

    ImGui::End();
}

void DesignImguiWrapper::DisplaySensorData() {
    ImGui::SetNextWindowSize(ImVec2(350.0f, 230.0f), ImGuiSetCond_Once);
    ImGui::SetNextWindowPos(ImVec2(20.0f, 160.0f), ImGuiSetCond_Once);
    ImGui::Begin("Sensor Data");
    
    std::stringstream aoa_str;
    aoa_str.precision(2);
    aoa_str.setf(std::ios::fixed, std::ios::floatfield);
    aoa_str << "AOA (degree): " << copter_utils::radian2degree(_parent_viewer->copter().angle_of_attack());
    ImGui::BulletText(aoa_str.str().c_str());

    // display IMU sensor data
    if (ImGui::CollapsingHeader("Ground Truth", ImGuiTreeNodeFlags_DefaultOpen)) {
        copter_simulation::InertialSensor& sensor =
            _parent_viewer->copter().clean_sensor();

        const Eigen::Vector3f pos = sensor.position();
        std::stringstream sensor_str;
        sensor_str.precision(3);
        sensor_str.setf(std::ios::fixed, std::ios::floatfield);
        sensor_str << "Position (m): " << pos.x() << ", "
            << pos.y() << ", " << pos.z();
        ImGui::BulletText(sensor_str.str().c_str());

        const Eigen::Vector3f rpy = 
            copter_utils::radian2degree(sensor.rpy());
        sensor_str.str("");
        sensor_str << "Euler Angle (degree): " << rpy(0) << ", "
            << rpy(1) << ", " << rpy(2);
        ImGui::BulletText(sensor_str.str().c_str());

        const Eigen::Vector3f velocity = sensor.velocity();
        sensor_str.str("");
        sensor_str << "Velocity (m/s): " << velocity.x() << ", "
            << velocity.y() << ", " << velocity.z();
        ImGui::BulletText(sensor_str.str().c_str());

        const Eigen::Vector3f euler_rate =
            copter_utils::radian2degree(sensor.rpy_rate());
        sensor_str.str("");
        sensor_str << "Euler Rate (degree/s): " << euler_rate(0) << ", "
            << euler_rate(1) << ", " << euler_rate(2);
        ImGui::BulletText(sensor_str.str().c_str());
    }
    
    if (ImGui::CollapsingHeader("Noisy Sensor",
        ImGuiTreeNodeFlags_DefaultOpen)) {
        copter_simulation::InertialSensor& sensor =
            _parent_viewer->copter().sensor();

        const Eigen::Vector3f pos = sensor.position();
        std::stringstream sensor_str;
        sensor_str.precision(3);
        sensor_str.setf(std::ios::fixed, std::ios::floatfield);
        sensor_str << "Position (m): " << pos.x() << ", "
            << pos.y() << ", " << pos.z();
        ImGui::BulletText(sensor_str.str().c_str());

        const Eigen::Vector3f rpy = 
            copter_utils::radian2degree(sensor.rpy());
        sensor_str.str("");
        sensor_str << "Euler Angle (degree): " << rpy(0) << ", "
            << rpy(1) << ", " << rpy(2);
        ImGui::BulletText(sensor_str.str().c_str());

        const Eigen::Vector3f velocity = sensor.velocity();
        sensor_str.str("");
        sensor_str << "Velocity (m/s): " << velocity.x() << ", "
            << velocity.y() << ", " << velocity.z();
        ImGui::BulletText(sensor_str.str().c_str());

        const Eigen::Vector3f euler_rate =
            copter_utils::radian2degree(sensor.rpy_rate());
        sensor_str.str("");
        sensor_str << "Euler Rate (degree/s): " << euler_rate(0) << ", "
            << euler_rate(1) << ", " << euler_rate(2);
        ImGui::BulletText(sensor_str.str().c_str());
    }
    ImGui::End();
}

void DesignImguiWrapper::DisplayCommandTarget() {
    ImGui::SetNextWindowSize(ImVec2(350.0f, 120.0f), ImGuiSetCond_Once);
    ImGui::SetNextWindowPos(ImVec2(20.0f, 650.0f), ImGuiSetCond_Once);
    ImGui::Begin("Command Target");

    std::stringstream control_mode_str;
    control_mode_str << "Control Mode: " << _control_mode;
    ImGui::BulletText(control_mode_str.str().c_str());

    std::stringstream command_target_str;
    command_target_str.precision(3);
    command_target_str.setf(std::ios::fixed, std::ios::floatfield);
    command_target_str << "Command Target: \n";
    for (int i = 0;i < _command_target.size();++i) {
        command_target_str << _command_target[i].first << ": " << _command_target[i].second;
        command_target_str << "\n";
    }
    ImGui::BulletText(command_target_str.str().c_str());

    ImGui::End();
}

}