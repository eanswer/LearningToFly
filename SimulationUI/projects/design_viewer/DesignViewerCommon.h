#pragma once
#include <string>

namespace design_viewer {
    // viewer update fps/frequency
    const int k_param_viewer_fps = 50;
    const float k_param_viwer_update_hz = 1.0f / (float)k_param_viewer_fps;
    const float k_param_simulation_dt = 0.0025f;

    // PROJECT SOURCE DIR
    const std::string k_param_project_source_dir = std::string(PROJECT_SOURCE_DIR);

    // config file root directory
    const std::string k_param_config_root_subdir = "/data/config/";

    // scene description file
    const std::string k_param_scene_path = "scene.txt";

    // radio
    const int k_param_radio_trim = 1500;
    const int k_param_radio_min = 1000;
    const int k_param_radio_max = 2000;
    const int k_param_radio_deadzone = 100;
    const int k_param_radio_step = 5;
};