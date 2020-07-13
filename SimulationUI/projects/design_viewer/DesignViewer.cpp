#include "DesignViewer.h"
#include "tiny_obj_loader.h"
#include <chrono>
#include <thread>

namespace design_viewer {

// public functions

DesignViewer::DesignViewer()
    : _copter(),
    _viewer(opengl_viewer::Viewer::GetViewer()),
    _imgui_wrapper(this),
    _focus_on_simulation(true),
    _paused(true),
    _up_pressed(false), _down_pressed(false),
    _left_pressed(false), _right_pressed(false),
    _w_pressed(false), _s_pressed(false),
    _a_pressed(false), _d_pressed(false),
    _e_pressed(false), _r_pressed(false),
    _mode_switch(0), _last_mode_switch(-1) {
}


DesignViewer::~DesignViewer() {
}

void DesignViewer::Initialize(const int fps) {

    // set up viewer options
    opengl_viewer::Option option;

    option.SetIntOption("height", 1200);
    option.SetIntOption("width", 2000);
    option.SetVectorOption("background color", 0.86f, 0.88f, 0.90f, 1.0f);
    option.SetFloatOption("camera aspect ratio", 1600.0f / 1000.0f);
    option.SetVectorOption("camera pos", -5.2f, 1.9f, -2.5f);
    option.SetVectorOption("camera look at", 0.3f, -0.2f, 0.0f);
    option.SetVectorOption("camera up", 0.0f, 0.0f, -1.0f);
    option.SetFloatOption("camera pan speed", 0.004f);
    option.SetFloatOption("camera field of view", 45.0f);
    option.SetFloatOption("shadow acne bias", 0.005f);
    option.SetFloatOption("shadow sampling angle", 0.2f);
    option.SetIntOption("shadow sampling number", 2);

    // Initialize Movable Camera handler
    _camera.Initialize(Eigen::Vector3f(-2.6f, 1.33f, -1.6f), Eigen::Vector3f(0.3f, -0.2f, 0.0f), this);
    option.SetPointerOption("camera", static_cast<void*>(&_camera));
    
    // Initialize keyboard handler
    _keyboard_handler.Initialize(true, this);
    option.SetPointerOption("keyboard handler", static_cast<void*>(&_keyboard_handler));

    // Initialize timer
    _timer.Initialize(fps, this);
    option.SetPointerOption("timer", static_cast<void*>(&_timer));

    // Initialize ImGui
    _imgui_wrapper.Initialize();
    _imgui_wrapper.add_status_data("system", "start");
    option.SetPointerOption("imgui wrapper",
        static_cast<void*>(&_imgui_wrapper));

    _viewer.Initialize(option);

    // Initialize Transmitter Input
    _rpyt_in(0) = _rpyt_in(1) = _rpyt_in(2) = k_param_radio_trim;
    _rpyt_in(3) = k_param_radio_min;

    // Add Ground
    AddGround();

    // Add Axis
    AddAxis();

    // initiate copter id
    _copter.LoadFromConfigFile(design_viewer::k_param_project_source_dir + design_viewer::k_param_config_root_subdir + "QuadPlane_config.txt");
    // _copter.LoadFromConfigFile(design_viewer::k_param_project_source_dir + design_viewer::k_param_config_root_subdir + "TailSitter_config.txt");
    LoadCopter();

    // Add Light
    AddLight();

    // Initialize Copter
    _copter.Initialize();

    // Initialize Scene
    if (_copter.director_mode())
        LoadScene(k_param_project_source_dir + k_param_config_root_subdir + k_param_scene_path);
}

void DesignViewer::Run() {
    _viewer.Run();
}

void DesignViewer::Cleanup() {
    _viewer.Cleanup();
}

void DesignViewer::LoadCopter() {
    AddCopter();

    AddArrows();
}

void DesignViewer::UpdateTransmitterInput() {

    if (!_paused) {
        if (_a_pressed && !_d_pressed) {
            _rpyt_in(2) = std::max(k_param_radio_min, _rpyt_in(2) - k_param_radio_step);
        }
        else if (!_a_pressed && _d_pressed) {
            _rpyt_in(2) = std::min(k_param_radio_max, _rpyt_in(2) + k_param_radio_step);
        }
        else if (!_a_pressed && !_d_pressed) {
            if (_rpyt_in(2) > k_param_radio_trim) {
                _rpyt_in(2) = std::max(k_param_radio_trim, _rpyt_in(2) - k_param_radio_step);
            }
            else {
                _rpyt_in(2) = std::min(k_param_radio_trim, _rpyt_in(2) + k_param_radio_step);
            }
        }

        if (!_up_pressed && _down_pressed) {
            _rpyt_in(1) = std::min(std::max(k_param_radio_min, _rpyt_in(1) + k_param_radio_step), k_param_radio_max);
        }
        else if (_up_pressed && !_down_pressed) {
            _rpyt_in(1) = std::max(std::min(k_param_radio_max, _rpyt_in(1) - k_param_radio_step), k_param_radio_min);
        }
        else if (!_up_pressed && !_down_pressed) {
            if (_rpyt_in(1) > k_param_radio_trim) {
                _rpyt_in(1) = std::max(k_param_radio_trim, _rpyt_in(1) - k_param_radio_step);
            }
            else {
                _rpyt_in(1) = std::min(k_param_radio_trim, _rpyt_in(1) + k_param_radio_step);
            }
        }

        if (_left_pressed && !_right_pressed) {
            _rpyt_in(0) = std::max(k_param_radio_min, _rpyt_in(0) - k_param_radio_step);
        }
        else if (!_left_pressed && _right_pressed) {
            _rpyt_in(0) = std::min(k_param_radio_max, _rpyt_in(0) + k_param_radio_step);
        }
        else if (!_left_pressed && !_right_pressed) {
            if (_rpyt_in(0) > k_param_radio_trim) {
                _rpyt_in(0) = std::max(k_param_radio_trim, _rpyt_in(0) - k_param_radio_step);
            }
            else {
                _rpyt_in(0) = std::min(k_param_radio_trim, _rpyt_in(0) + k_param_radio_step);
            }
        }

        if (_w_pressed && !_s_pressed) {
            _rpyt_in(3) = std::min(k_param_radio_max, _rpyt_in(3) + k_param_radio_step);
        }
        else if (!_w_pressed && _s_pressed) {
            _rpyt_in(3) = std::max(k_param_radio_min, _rpyt_in(3) - k_param_radio_step);
        }

        // Reset target speed for NN controller
        if (_copter.flight_controller() == copter_simulation::FlightController::NN) {
            if (_e_pressed)
                _rpyt_in(1) = k_param_radio_trim;
            if (_r_pressed)
                _rpyt_in(3) = k_param_radio_trim;
        }
    }

    // pass input targets to controller
    _copter.SetRadioInput(_rpyt_in, _mode_switch);

    // display mode switch for quadplane controller
    if (_copter.flight_controller() == copter_simulation::FlightController::QuadPlane)
        if (_mode_switch != _last_mode_switch) {
            if (_copter.flight_mode() == copter_simulation::FlightMode::COPTER_MODE) {
                _imgui_wrapper.add_status_data("control", "enter COPTER mode");
            }
            else {
                _imgui_wrapper.add_status_data("control", "enter GLIDING mode");
            }
            _last_mode_switch = _mode_switch;
        }

    // display controller
    switch (_copter.flight_controller()) {
    case copter_simulation::FlightController::QuadPlane:
        _imgui_wrapper.set_control_mode("PID Controller");
        break;
    case copter_simulation::FlightController::NN:
        _imgui_wrapper.set_control_mode("NN Controller");
        break;
    }

    // print command data
    std::vector<std::pair<std::string, float> > command_target;
    command_target.clear();
    switch (_copter.flight_controller()) {
    case copter_simulation::FlightController::QuadPlane:
        if (_copter.flight_mode() == copter_simulation::FlightMode::COPTER_MODE) {
            command_target.push_back(std::make_pair("target roll(degree)    ", copter_utils::radian2degree(_copter.roll_in())));
            command_target.push_back(std::make_pair("target pitch(degree)   ", copter_utils::radian2degree(_copter.pitch_in())));
            command_target.push_back(std::make_pair("target yaw rate(degree)", copter_utils::radian2degree(_copter.yaw_in())));
            command_target.push_back(std::make_pair("target altitude rate   ", _copter.throttle_in()));
        }
        else {
            command_target.push_back(std::make_pair("target roll(degree)    ", copter_utils::radian2degree(_copter.roll_in())));
            command_target.push_back(std::make_pair("target pitch(degree)   ", copter_utils::radian2degree(_copter.pitch_in())));
            command_target.push_back(std::make_pair("target yaw rate(degree)", copter_utils::radian2degree(_copter.yaw_in())));
            command_target.push_back(std::make_pair("target throttle        ", _copter.throttle_in()));
        }
        break;
    case copter_simulation::FlightController::NN:
        _imgui_wrapper.set_control_mode("NN Controller");
        command_target.push_back(std::make_pair("target velocity x      ", _copter.vx_in()));
        command_target.push_back(std::make_pair("target velocity y      ", _copter.vy_in()));
        command_target.push_back(std::make_pair("target velocity z      ", _copter.vz_in()));
        command_target.push_back(std::make_pair("target yaw             ", _copter.target_yaw_diff()));
        break;
    }
    _imgui_wrapper.set_command_target(command_target);
}

void DesignViewer::Advance(float dt) {
    // clock synchronization
    auto t_start = std::chrono::high_resolution_clock::now();
    float total_time = 0.f;
    for (int step = 0; step < (int)(dt / k_param_simulation_dt); step++) {
        _copter.Advance(k_param_simulation_dt);
        total_time += k_param_simulation_dt;
    }
    if (total_time + 1e-3f < dt)
        _copter.Advance(dt - total_time);
    auto t_end = std::chrono::high_resolution_clock::now();
    auto t_span = std::chrono::duration_cast<std::chrono::duration<float>>(t_end - t_start);
    std::this_thread::sleep_for(std::chrono::milliseconds((int)((dt - t_span.count()) * 1e3f)));

    UpdateUI();
}

Eigen::Vector3f DesignViewer::copter_position() {
    return _copter.rigid_body().position().cast<float>();
}

void DesignViewer::flip_paused() {
    _paused = !_paused;
    if (!_paused) {
        _imgui_wrapper.add_status_data("simulation", "start simulation");
    } else {
        _imgui_wrapper.add_status_data("simulation", "pause simulation");
    }
}
// private functions

void DesignViewer::AddGround() {
    
    // Generate the ground texture.
    const int texture_size = 2, square_size = 1;
    std::vector<std::vector<Eigen::Vector3f>> texture_image(texture_size,
        std::vector<Eigen::Vector3f>(texture_size, Eigen::Vector3f::Zero()));
    for (int i = 0; i < texture_size; ++i) {
        for (int j = 0; j < texture_size; ++j) {
            // Determine the color of the checker.
            if ((i / square_size - j / square_size) % 2) {
                texture_image[i][j] = Eigen::Vector3f(157.0f, 150.0f, 143.0f) / 255.0f;
            }
            else {
                texture_image[i][j] = Eigen::Vector3f(216.0f, 208.0f, 197.0f) / 255.0f;
            }
        }
    }
    opengl_viewer::Image checker;
    checker.Initialize(texture_image);

    Eigen::Matrix3Xf ground_vertex = Eigen::Matrix3Xf::Zero(3, 4);
    ground_vertex << -1.0f, -1.0f, 1.0f, 1.0f,
        -1.0f, 1.0f, -1.0f, 1.0f,
        0.0f, 0.0f, 0.0f, 0.0f;
    Eigen::Matrix3Xi ground_face = Eigen::Matrix3Xi::Zero(3, 2);
    ground_face << 0, 2, 1, 1, 2, 3;
    const float scale = 1000.0f;
    Eigen::Matrix2Xf ground_uv = ground_vertex.topRows(2) / 2.0f * scale;
    opengl_viewer::Option ground_option;
    ground_option.SetMatrixOption("model matrix",
        opengl_viewer::Scale(scale, scale, 1.0f));
    ground_option.SetVectorOption("ambient", 0.75f, 0.77f, 0.72f);
    ground_option.SetVectorOption("diffuse", 0.84f, 0.80f, 0.81f);
    ground_option.SetIntOption("texture row num", texture_size);
    ground_option.SetIntOption("texture col num", texture_size);
    ground_option.SetMatrixOption("texture", checker.rgb_data());
    ground_option.SetMatrixOption("uv", ground_uv);
    ground_option.SetStringOption("texture mag filter", "nearest");
    _viewer.AddStaticObject(ground_vertex, ground_face, ground_option);
}

void DesignViewer::AddAxis() {
    std::vector<Eigen::Matrix3Xf> axis_vertices(2);
    std::vector<Eigen::Matrix3Xi> axis_faces(2);
    opengl_viewer::Cylinder(0.01f, 0.01f, 0.9f, 16, 1,
        axis_vertices[0], axis_faces[0]);
    opengl_viewer::Cone(0.02f, 0.15f, 16, 1, axis_vertices[1], axis_faces[1]);
    opengl_viewer::Translate(0.0f, 0.0f, 0.85f, axis_vertices[1]);
    // Merge them into a single arrow.
    Eigen::Matrix3Xf axis_vertex;
    Eigen::Matrix3Xi axis_face;
    opengl_viewer::MergeMeshes(axis_vertices, axis_faces,
        axis_vertex, axis_face);
    // Now add x axis.
    opengl_viewer::Option axis_option;
    axis_option.SetMatrixOption("model matrix",
        opengl_viewer::Rotate(90.0f, 0.0f, 1.0f, 0.0f));
    axis_option.SetVectorOption("ambient", 1.0f, 0.0f, 0.0f);
    axis_option.SetVectorOption("diffuse", 1.0f, 0.0f, 0.0f);
    axis_option.SetVectorOption("specular", 1.0f, 0.0f, 0.0f);
    _viewer.AddStaticObject(axis_vertex, axis_face, axis_option);
    
    // Then y axis.
    axis_option.SetMatrixOption("model matrix",
        opengl_viewer::Rotate(90.0f, -1.0f, 0.0f, 0.0f));
    axis_option.SetVectorOption("ambient", 0.0f, 1.0f, 0.0f);
    axis_option.SetVectorOption("diffuse", 0.0f, 1.0f, 0.0f);
    axis_option.SetVectorOption("specular", 0.0f, 1.0f, 0.0f);
    _viewer.AddStaticObject(axis_vertex, axis_face, axis_option);
}

void DesignViewer::AddLight() {
    opengl_viewer::Option light_option;
    light_option.SetVectorOption("ambient", 0.54f, 0.56f, 0.51f);
    light_option.SetVectorOption("diffuse", 0.55f, 0.59f, 0.56f);
    light_option.SetVectorOption("specular", 0.21f, 0.26f, 0.24f);
    // Animate the light.
    const Eigen::Vector3f initial_position(0.2f, -0.3f, -1.0f);
    _light_animator.Initialize(initial_position, this);
    _viewer.AddDynamicPointLight(&_light_animator, light_option);
}

void DesignViewer::AddCopter() {
    if (_current_object_id.size() > 0) {
        for (int i = 0;i < _current_object_id.size();++i)
            _viewer.RemoveObject(_current_object_id[i]);
        _copter_animators.clear();
        _current_object_id.clear();
    }

    Eigen::Matrix3Xf vertices;
    Eigen::Matrix3Xi elements;

    opengl_viewer::Option copter_option;
    copter_option.Clear();
    copter_option.SetVectorOption("ambient", 0.147f, 0.166f, 0.989f);
    copter_option.SetVectorOption("diffuse", 0.302f, 0.876f, 0.727f);
    copter_option.SetVectorOption("specular", 0.376f, 0.093f, 0.677f);

    int body_mesh_size = _copter.copter_mesh().body_mesh().size();
    _copter_animators = std::vector<CopterAnimator>(body_mesh_size, CopterAnimator());

    // Add copter body mesh parts
    for (int i = 0; i < body_mesh_size; ++i) {
        copter_utils::SurfaceMesh& mesh = _copter.copter_mesh().body_mesh(i);
        GenerateCopterViewerObject(mesh, vertices, elements);

        _copter_animators[i].Initialize(i, this);

        _current_object_id.push_back(_viewer.AddDynamicObject(vertices, elements, &_copter_animators[i], copter_option));
    }

    // Add rotors and propellers (default: counter-clockwise)
    std::string rotor_mesh_file = design_viewer::k_param_project_source_dir + design_viewer::k_param_config_root_subdir + "rotor.obj";
    GenerateCopterViewerObject(rotor_mesh_file.c_str(), vertices, elements);

    int rotor_num = _copter.rotor_num();
    _copter_rotor_animators = std::vector<CopterRotorAnimator>(rotor_num * 2, CopterRotorAnimator());

    opengl_viewer::Option copter_rotor_option;
    copter_rotor_option.Clear();
    copter_rotor_option.SetVectorOption("ambient", 0.147f, 0.989f, 0.166f);
    copter_rotor_option.SetVectorOption("diffuse", 0.302f, 0.876f, 0.727f);
    copter_rotor_option.SetVectorOption("specular", 0.376f, 0.093f, 0.677f);

    for (int i = 0; i < rotor_num; ++i) {
        _copter_rotor_animators[i].Initialize(i, this);
        _current_object_id.push_back(_viewer.AddDynamicObject(vertices, elements, &_copter_rotor_animators[i], copter_option));
    }

    std::string propeller_mesh_file = design_viewer::k_param_project_source_dir + design_viewer::k_param_config_root_subdir + "propeller.obj";
    GenerateCopterViewerObject(propeller_mesh_file.c_str(), vertices, elements);
    
    // Create propeller for clockwise rotors
    Eigen::Matrix3Xf vertices_mirror(vertices);
    Eigen::Matrix3Xi elements_mirror(elements);
    vertices_mirror.row(0) = -vertices_mirror.row(0);
    elements_mirror.row(0).swap(elements_mirror.row(1));

    for (int i = 0; i < rotor_num; ++i) {
        _copter_rotor_animators[rotor_num + i].Initialize(i, this, true);
        _current_object_id.push_back(_viewer.AddDynamicObject(
            _copter.rotors()[i].is_ccw() ? vertices : vertices_mirror,
            _copter.rotors()[i].is_ccw() ? elements : elements_mirror,
            &_copter_rotor_animators[rotor_num + i], copter_rotor_option));
    }

    // Reset Simulation
    // _copter.Reset();
}

void DesignViewer::AddArrows() {
    if (_current_arrow_object_id.size() > 0) {
        for (int i = 0;i < _current_arrow_object_id.size();++i)
            _viewer.RemoveObject(_current_arrow_object_id[i]);
        _arrow_animators.clear();
        _current_arrow_object_id.clear();
    }

    _arrow_animators = std::vector<ArrowAnimator>(_copter.rotor_num() + 3, ArrowAnimator());

    AddThrust();

    AddAerodynamics();
}

void DesignViewer::AddThrust() {    
    Eigen::Matrix3Xf arrow_vertex;
    Eigen::Matrix3Xi arrow_face;
    opengl_viewer::Arrow(0.01f, 0.5f, Eigen::Vector3f::UnitZ(), arrow_vertex, arrow_face);

    opengl_viewer::Option arrow_option;
    arrow_option.SetVectorOption("ambient", 0.5f, 0.5f, 0.5f);
    arrow_option.SetVectorOption("diffuse", 0.5f, 0.5f, 0.5f);
    arrow_option.SetVectorOption("specular", 0.5f, 0.5f, 0.5f);
    
    for (int i = 0;i < _copter.rotors().size();++i) {
        _current_arrow_object_id.push_back(
            _viewer.AddStaticObject(arrow_vertex, arrow_face, arrow_option));
    }
}

void DesignViewer::AddAerodynamics() {
    Eigen::Matrix3Xf arrow_vertex;
    Eigen::Matrix3Xi arrow_face;
    opengl_viewer::Arrow(0.01f, 0.5f, Eigen::Vector3f::UnitZ(), arrow_vertex, arrow_face);

    opengl_viewer::Option arrow_option;

    int idx = _copter.rotor_num();

    // velocity
    arrow_option.SetVectorOption("ambient", 0.8f, 0.8f, 0.0f);
    arrow_option.SetVectorOption("diffuse", 0.8f, 0.8f, 0.0f);
    arrow_option.SetVectorOption("specular", 0.8f, 0.8f, 0.0f);

    _current_arrow_object_id.push_back(_viewer.AddStaticObject(arrow_vertex, arrow_face, arrow_option));
    
    // target velocity
    ++idx;
    arrow_option.SetVectorOption("ambient", 0.8f, 0.0f, 0.0f);
    arrow_option.SetVectorOption("diffuse", 0.8f, 0.0f, 0.0f);
    arrow_option.SetVectorOption("specular", 0.8f, 0.0f, 0.0f);

    _current_arrow_object_id.push_back(_viewer.AddStaticObject(arrow_vertex, arrow_face, arrow_option));

    // // lift
    // ++idx;
    // arrow_option.SetVectorOption("ambient", 0.0f, 0.8f, 0.0f);
    // arrow_option.SetVectorOption("diffuse", 0.0f, 0.8f, 0.0f);
    // arrow_option.SetVectorOption("specular", 0.0f, 0.8f, 0.0f);

    // _current_arrow_object_id.push_back(_viewer.AddStaticObject(arrow_vertex, arrow_face, arrow_option));

    // // drag
    // ++idx;
    // arrow_option.SetVectorOption("ambient", 0.8f, 0.0f, 0.0f);
    // arrow_option.SetVectorOption("diffuse", 0.8f, 0.0f, 0.0f);
    // arrow_option.SetVectorOption("specular", 0.8f, 0.0f, 0.0f);

    // _current_arrow_object_id.push_back(_viewer.AddStaticObject(arrow_vertex, arrow_face, arrow_option));
}

void DesignViewer::UpdateUI() {
    _imgui_wrapper.set_current_time(_copter.current_time());

    UpdateArrows();
}

void DesignViewer::UpdateArrows() {
    UpdateThrust();

    UpdateAerodynamics();
}

void DesignViewer::UpdateThrust() {
    for (int i = 0;i < _copter.rotor_num();++i) {
        float desired_length = _copter.rotors(i).get_thrust().norm() / copter_simulation::k_param_rotor_max_thrust * 0.5;
        
        Eigen::Matrix3Xf arrow_vertex;
        Eigen::Matrix3Xi arrow_face;
        
        opengl_viewer::Arrow(0.01f, desired_length, _copter.rotors(i).direction_body_frame(), arrow_vertex, arrow_face);

        opengl_viewer::Translate(_copter.rotors(i).position_body_frame(), arrow_vertex);

        opengl_viewer::Rotate(_copter.rigid_body().orientation().cast<float>(), arrow_vertex);

        opengl_viewer::Translate(_copter.rigid_body().position().cast<float>(), arrow_vertex);

        opengl_viewer::Option arrow_option;
        arrow_option.SetVectorOption("ambient", 0.5f, 0.5f, 0.5f);
        arrow_option.SetVectorOption("diffuse", 0.5f, 0.5f, 0.5f);
        arrow_option.SetVectorOption("specular", 0.5f, 0.5f, 0.5f);

        _viewer.UpdateStaticObject(_current_arrow_object_id[i], arrow_vertex, arrow_face, arrow_option);
    }
}

void DesignViewer::UpdateAerodynamics() {
    int idx = _copter.rotor_num() - 1;
    float desired_length;
    opengl_viewer::Option arrow_option;

    // velocity
    {
        ++idx;
        Eigen::Vector3f velocity = _copter.rigid_body().velocity().cast<float>();
        Eigen::Vector3f velocity_dir = velocity;

        velocity_dir.normalize();
        desired_length = velocity.norm() / 4.0f;

        Eigen::Matrix3Xf arrow_vertex;
        Eigen::Matrix3Xi arrow_face;

        opengl_viewer::Arrow(0.01f, desired_length, velocity_dir, arrow_vertex, arrow_face);

        opengl_viewer::Translate(_copter.rigid_body().position().cast<float>(), arrow_vertex);

        arrow_option.SetVectorOption("ambient", 0.8f, 0.8f, 0.0f);
        arrow_option.SetVectorOption("diffuse", 0.8f, 0.8f, 0.0f);
        arrow_option.SetVectorOption("specular", 0.8f, 0.8f, 0.0f);

        _viewer.UpdateStaticObject(_current_arrow_object_id[idx], arrow_vertex, arrow_face, arrow_option);
    }

    // target velocity
    {
        ++idx;
        Eigen::Vector3f target_velocity = _copter.current_target();
        Eigen::Vector3f target_velocity_dir = target_velocity;

        target_velocity_dir.normalize();
        desired_length = target_velocity.norm() / 2.0f;
        
        Eigen::Matrix3Xf arrow_vertex;
        Eigen::Matrix3Xi arrow_face;

        opengl_viewer::Arrow(0.01f, desired_length, target_velocity_dir, arrow_vertex, arrow_face);

        opengl_viewer::Translate(_copter.rigid_body().position().cast<float>(), arrow_vertex);

        arrow_option.SetVectorOption("ambient", 0.8f, 0.0f, 0.0f);
        arrow_option.SetVectorOption("diffuse", 0.8f, 0.0f, 0.0f);
        arrow_option.SetVectorOption("specular", 0.8f, 0.0f, 0.0f);
        
        _viewer.UpdateStaticObject(_current_arrow_object_id[idx], arrow_vertex, arrow_face, arrow_option);
    }

    // // lift
    // {
    //     ++idx;
    //     Eigen::Vector3f lift = _copter.lift_force();
    //     Eigen::Vector3f lift_dir = lift;
        
    //     lift_dir.normalize();
    //     desired_length = lift.norm() / copter_simulation::k_param_rotor_max_thrust;

    //     Eigen::Matrix3Xf arrow_vertex;
    //     Eigen::Matrix3Xi arrow_face;

    //     opengl_viewer::Arrow(0.01f, desired_length, lift_dir, arrow_vertex, arrow_face);

    //     opengl_viewer::Translate(_copter.rigid_body().position().cast<float>(), arrow_vertex);

    //     arrow_option.SetVectorOption("ambient", 0.0f, 0.8f, 0.0f);
    //     arrow_option.SetVectorOption("diffuse", 0.0f, 0.8f, 0.0f);
    //     arrow_option.SetVectorOption("specular", 0.0f, 0.8f, 0.0f);

    //     _viewer.UpdateStaticObject(_current_arrow_object_id[idx], arrow_vertex, arrow_face, arrow_option);
    // }

    // // drag
    // {
    //     ++idx;
    //     Eigen::Vector3f drag = _copter.drag_force();
    //     Eigen::Vector3f drag_dir = drag;

    //     drag_dir.normalize();
    //     desired_length = drag.norm() / copter_simulation::k_param_rotor_max_thrust;

    //     Eigen::Matrix3Xf arrow_vertex;
    //     Eigen::Matrix3Xi arrow_face;

    //     opengl_viewer::Arrow(0.01f, desired_length, drag_dir, arrow_vertex, arrow_face);

    //     opengl_viewer::Translate(_copter.rigid_body().position().cast<float>(), arrow_vertex);

    //     arrow_option.SetVectorOption("ambient", 0.8f, 0.0f, 0.0f);
    //     arrow_option.SetVectorOption("diffuse", 0.8f, 0.0f, 0.0f);
    //     arrow_option.SetVectorOption("specular", 0.8f, 0.0f, 0.0f);

    //     _viewer.UpdateStaticObject(_current_arrow_object_id[idx], arrow_vertex, arrow_face, arrow_option);
    // }
}

void DesignViewer::GenerateCopterViewerObject(copter_utils::SurfaceMesh& mesh,
    Eigen::Matrix3Xf& vertices, Eigen::Matrix3Xi& elements) {

    std::vector<float> all_vertices(0);
    std::vector<int> all_faces(0);
    int vertex_num = 0, face_num = 0;
    for (int j = 0;j < mesh.vertices().size();++j) {
        all_vertices.push_back(mesh.vertices(j).x());
        all_vertices.push_back(mesh.vertices(j).y());
        all_vertices.push_back(mesh.vertices(j).z());
        ++vertex_num;
    }
    for (int j = 0;j < mesh.elements().size();++j) {
        all_faces.push_back(mesh.elements(j).x());
        all_faces.push_back(mesh.elements(j).y());
        all_faces.push_back(mesh.elements(j).z());
        ++face_num;
    }

    vertices = Eigen::Map<Eigen::Matrix3Xf>(all_vertices.data(), 3, vertex_num);
    elements = Eigen::Map<Eigen::Matrix3Xi>(all_faces.data(), 3, face_num);
}

void DesignViewer::GenerateCopterViewerObject(const char *file_name,
    Eigen::Matrix3Xf& vertices, Eigen::Matrix3Xi& elements) {
    
    std::vector<tinyobj::shape_t> obj_shape;
    std::vector<tinyobj::material_t> obj_material;
    tinyobj::attrib_t attrib;
    std::string err;
    tinyobj::LoadObj(&attrib, &obj_shape, &obj_material, &err, file_name);

    vertices.resize(3, attrib.vertices.size() / 3);
    elements.resize(3, obj_shape[0].mesh.indices.size() / 3);
    for (int i = 0; i < vertices.cols(); ++i) {
        vertices.col(i)[0] = attrib.vertices[i * 3];
        vertices.col(i)[1] = attrib.vertices[i * 3 + 1];
        vertices.col(i)[2] = attrib.vertices[i * 3 + 2];
    }
    auto &indices = obj_shape[0].mesh.indices;
    for (int i = 0; i < elements.cols(); ++i) {
        elements.col(i)[0] = indices[i * 3].vertex_index;
        elements.col(i)[1] = indices[i * 3 + 1].vertex_index;
        elements.col(i)[2] = indices[i * 3 + 2].vertex_index;
    }
}

bool DesignViewer::LoadScene(const std::string &scene_file_name) {
    // Read from scene description file
    std::ifstream ifs(scene_file_name);
    if (!ifs.is_open())
        return false;
    
    // Create a unit cube
    Eigen::Matrix3Xf cube_v(3, 8);
    cube_v << -0.5f, -0.5f, -0.5f, -0.5f, 0.5f, 0.5f, 0.5f, 0.5f,
              -0.5f, -0.5f, 0.5f, 0.5f, -0.5f, -0.5f, 0.5f, 0.5f,
              0.0f, -1.0f, 0.0f, -1.0f, 0.0f, -1.0f, 0.0f, -1.0f;
    Eigen::Matrix3Xi cube_e(3, 12);
    cube_e << 0, 3, 0, 5, 0, 6, 1, 7, 2, 7, 4, 7,
              2, 1, 1, 4, 4, 2, 3, 5, 6, 3, 5, 6,
              1, 2, 4, 1, 2, 4, 5, 3, 3, 6, 6, 5;
    Eigen::Matrix3Xf cuboid_v(cube_v);

    Eigen::Vector3f pos, size, color;
    float angle;
    opengl_viewer::Option option;

    _current_scene_object_id.clear();
    while (ifs >> pos(0) >> pos(1) >> pos(2) >>
        size(0) >> size(1) >> size(2) >> angle >>
        color(0) >> color(1) >> color(2)) {

        option.SetVectorOption("ambient", color(0), color(1), color(2));
        option.SetVectorOption("diffuse", color(0), color(1), color(2));
        option.SetVectorOption("specular", color(0), color(1), color(2));

        Eigen::Matrix3f rotation = Eigen::AngleAxisf(angle * M_PI / 180.0f, -Eigen::Vector3f::UnitZ()).toRotationMatrix();
        for (int i = 0; i < 8; ++i)
            cuboid_v.col(i) = rotation * cube_v.col(i).cwiseProduct(size) + pos;

        // Place a static cuboid object in the scene
        _current_scene_object_id.push_back(_viewer.AddStaticObject(cuboid_v, cube_e, option));
    }

    return true;
}

}