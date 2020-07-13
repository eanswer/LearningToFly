#pragma once
#include "opengl_viewer.h"
#include "DesignImguiWrapper.h"
#include "HybridCopter.h"
#include "DesignViewerKeyboardHandler.h"
#include "DesignViewerTimer.h"
#include "DesignViewerCamera.h"
#include "CopterAnimator.h"
#include "CopterRotorAnimator.h"
#include "LightAnimator.h"
#include "ArrowAnimator.h"

namespace design_viewer {

class DesignViewer
{
public:
    DesignViewer();
    ~DesignViewer();

    void Initialize(const int fps);
    void Run();
    void Cleanup();
    
    void LoadCopter();
    
    // Director's Mode
    bool LoadScene(const std::string &scene_file_name);

    void UpdateTransmitterInput();

    // simulation
    void Advance(float dt);

    // get functions of keyboard_hander's variables
    copter_simulation::HybridCopter& copter() { return _copter; }
    bool paused() { return _paused; }
    bool up_pressed() { return _up_pressed; }
    bool down_pressed() { return _down_pressed; }
    bool left_pressed() { return _left_pressed; }
    bool right_pressed() { return _right_pressed; }
    bool w_pressed() { return _w_pressed; }
    bool s_pressed() { return _s_pressed; }
    bool a_pressed() { return _a_pressed; }
    bool d_pressed() { return _d_pressed; }
    bool e_pressed() { return _e_pressed; } // Reset key for vx
    bool r_pressed() { return _r_pressed; } // Reset key for vz
    bool focus_on_simulation() { return _focus_on_simulation; }
    DesignImguiWrapper& imgui_wrapper() { return _imgui_wrapper; }
    Eigen::Vector3f copter_position();

    // set functions of keyboard_hander's variables
    void set_paused(bool flag) { _paused = flag; }
    void flip_paused();
    void set_up_pressed(bool flag) { _up_pressed = flag; }
    void set_down_pressed(bool flag) { _down_pressed = flag; }
    void set_left_pressed(bool flag) { _left_pressed = flag; }
    void set_right_pressed(bool flag) { _right_pressed = flag; }
    void set_w_pressed(bool flag) { _w_pressed = flag; }
    void set_s_pressed(bool flag) { _s_pressed = flag; }
    void set_a_pressed(bool flag) { _a_pressed = flag; }
    void set_d_pressed(bool flag) { _d_pressed = flag; }
    void set_e_pressed(bool flag) { _e_pressed = flag; }
    void set_r_pressed(bool flag) { _r_pressed = flag; }
    void set_mode_switch(int mode_switch) { _mode_switch = mode_switch; }
    void set_focus_on_simulation(bool focus_on_simulation) { _focus_on_simulation = focus_on_simulation; }
    void switch_controller() { _copter.SwitchController(); }

private:
    void AddGround();
    void AddAxis();
    void AddLight();
    void AddCopter();
    void AddArrows();
    void AddThrust();
    void AddAerodynamics();

    void UpdateUI();
    void UpdateArrows();
    void UpdateThrust();
    void UpdateAerodynamics();

    void GenerateCopterViewerObject(copter_utils::SurfaceMesh& mesh,
        Eigen::Matrix3Xf& vertices, Eigen::Matrix3Xi& elements);
    void GenerateCopterViewerObject(const char *file_name,
        Eigen::Matrix3Xf& vertices, Eigen::Matrix3Xi& elements);

    opengl_viewer::Viewer& _viewer;
    DesignImguiWrapper _imgui_wrapper;
    DesignViewerKeyboardHandler _keyboard_handler;
    DesignViewerTimer _timer;
    DesignViewerCamera _camera;

    copter_simulation::HybridCopter _copter;
    
    // If focus on simulation panel
    bool _focus_on_simulation;

    // Variables changed by keyboard_handler
    bool _paused;
    bool _up_pressed, _down_pressed, _left_pressed, _right_pressed;
    bool _w_pressed, _s_pressed, _a_pressed, _d_pressed;
    bool _e_pressed, _r_pressed;
    int _mode_switch, _last_mode_switch;

    // Radio Transmitter Input
    Eigen::Vector4i _rpyt_in;

    // copter display related
    std::vector<int> _current_object_id;
    std::vector<CopterAnimator> _copter_animators;
    std::vector<CopterRotorAnimator> _copter_rotor_animators;

    // arrow display related (i.e. force)
    // first the rotor thrusts
    // then velocity, lift, drag
    std::vector<int> _current_arrow_object_id;
    std::vector<ArrowAnimator> _arrow_animators;

    // Scene
    std::vector<int> _current_scene_object_id;

    LightAnimator _light_animator;
};

}

