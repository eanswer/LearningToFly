#include "opengl_viewer.h"
#include <iostream>

namespace test_opengl_viewer {

    class MovingMeshKeyboardHandler : public opengl_viewer::KeyboardHandler {
    public:
        MovingMeshKeyboardHandler()
            : opengl_viewer::KeyboardHandler(), _paused(false) {}

        void Initialize(const bool paused) { _paused = paused; }

        void KeyCallback(const int key, const int action) {
            if (key == GLFW_KEY_P && action == GLFW_PRESS) {
                _paused = !_paused;
            }
        }

        const bool paused() const { return _paused; }

    private:
        bool _paused;
    };

    class MovingMeshTimer : public opengl_viewer::Timer {
    public:
        MovingMeshTimer()
            : opengl_viewer::Timer(), _fps(25), _dt(0.04f), _current_time(0.0f),
            _keyboard_handler(NULL) {}

        void Initialize(const int fps,
            const MovingMeshKeyboardHandler* keyboard_handler) {
            _fps = fps;
            _dt = 1.0f / fps;
            _current_time = 0.0f;
            _keyboard_handler = keyboard_handler;
        }

        const float CurrentTime() {
            if (!_keyboard_handler->paused()) _current_time += _dt;
            return _current_time;
        }

    private:
        int _fps;
        float _dt;
        float _current_time;
        const MovingMeshKeyboardHandler* _keyboard_handler;
    };

    class MovingMeshAnimator : public opengl_viewer::Animator {
    public:
        MovingMeshAnimator() :
            Animator() {}

        const Eigen::Matrix4f AnimatedModelMatrix(
            const float t) {
            return opengl_viewer::Translate(4 * std::sin(0.1 * t) * Eigen::Vector3f::UnitZ())
                * opengl_viewer::Rotate(10.0f * t, Eigen::Vector3f::UnitY());
        }

    };

    class MovingMeshLightAnimator : public opengl_viewer::Animator {
    public:
        MovingMeshLightAnimator() :
            Animator() {}

        const Eigen::Matrix4f AnimatedModelMatrix(
            const float t) {
            return opengl_viewer::Translate(4 * std::sin(0.1 * t) * Eigen::Vector3f::UnitZ() + Eigen::Vector3f::UnitY())
                * opengl_viewer::Rotate(10.0f * t, Eigen::Vector3f::UnitY());
        }
    };

    void moving_mesh() {
        opengl_viewer::Viewer& viewer = opengl_viewer::Viewer::GetViewer();
        opengl_viewer::Option option;
        option.SetFloatOption("camera field of view", 60.0f);
        option.SetFloatOption("camera aspect ratio", 4.0f / 3.0f);
        option.SetIntOption("height", 768);
        option.SetIntOption("width", 1024);
        option.SetFloatOption("shadow acne bias", 0.005f);
        option.SetFloatOption("shadow sampling angle", 0.57f);
        option.SetIntOption("shadow sampling number", 2);

        MovingMeshKeyboardHandler key_handler;
        key_handler.Initialize(false);
        MovingMeshTimer timer;
        timer.Initialize(25, &key_handler);
        option.SetPointerOption("timer", static_cast<void*>(&timer));
        option.SetPointerOption("keyboard handler", static_cast<void*>(&key_handler));
        opengl_viewer::SampleImGuiWrapper imgui_wrapper;
        option.SetPointerOption("imgui wrapper", static_cast<void*>(&imgui_wrapper));

        viewer.Initialize(option);

        const int ground_size = 2048, stripe_size = 128;
        std::vector<std::vector<Eigen::Vector3f>> ground_image(ground_size);
        for (int i = 0;i < ground_size;++i) {
            ground_image[i] = std::vector<Eigen::Vector3f>(ground_size);
            for (int j = 0;j < ground_size;++j) {
                if ((j / stripe_size) % 2 == 0) {
                    ground_image[i][j] = Eigen::Vector3f(255.0f, 255.0f, 255.0f) / 255.0f;
                }
                else {
                    ground_image[i][j] = Eigen::Vector3f(0.0f, 0.0f, 0.0f) / 255.0f;
                }
            }
        }
        opengl_viewer::Image ground_texture;
        ground_texture.Initialize(ground_image);

        const Eigen::Matrix3Xf vertex = (Eigen::Matrix<float, 3, 4>()
            << -1.0f, 1.0f, -1.0f, 1.0f,
            0.0f, 0.0f, 0.0f, 0.0f,
            -1.0f, -1.0f, 1.0f, 1.0f).finished();
        const Eigen::Matrix3Xi face = (Eigen::Matrix<int, 3, 2>()
            << 0, 0,
            2, 3,
            3, 1).finished();
        const Eigen::Matrix2Xf uv = (Eigen::Matrix<float, 2, 4>()
            << 0.0f, 0.0f, 1.0f, 1.0f,
            0.0f, 1.0f, 0.0f, 1.0f).finished();
        const Eigen::Matrix4f scale_matrix =
            Eigen::Vector4f(4.0f, 1.0f, 4.0f, 1.0f).asDiagonal();
        opengl_viewer::Option object_option;
        object_option.SetMatrixOption("model matrix", scale_matrix);
        object_option.SetVectorOption("ambient", 0.7f, 0.7f, 0.7f);
        object_option.SetVectorOption("diffuse", 0.8f, 0.8f, 0.8f);
        object_option.SetVectorOption("specular", 1.0f, 1.0f, 1.0f);
        object_option.SetFloatOption("shininess", 1.5f);
        object_option.SetMatrixOption("uv", uv);
        object_option.SetMatrixOption("texture", ground_texture.rgb_data());
        object_option.SetIntOption("texture row num", ground_size);
        object_option.SetIntOption("texture col num", ground_size);
        object_option.SetStringOption("texture mag filter", "nearest");
        viewer.AddStaticObject(vertex, face, object_option);

        // input a mesh
        std::string mesh_filename = "D:\\Projects\\UAVs\\HybridUAV_design\\data\\plane.obj";

        Eigen::Matrix3Xf vertices;
        Eigen::Matrix2Xf dummy_uv;
        Eigen::Matrix3Xi elements;
        opengl_viewer::ReadFromObjFile(mesh_filename, vertices, elements, dummy_uv);

        opengl_viewer::Rotate(90.0f, Eigen::Vector3f::UnitX(), vertices);
        opengl_viewer::Rotate(90.0f, Eigen::Vector3f::UnitY(), vertices);
        opengl_viewer::Translate(Eigen::Vector3f(0.0, 0.1, 0.0), vertices);

        object_option.Clear();
        object_option.SetVectorOption("ambient", 0.147f, 0.166f, 0.989f);
        object_option.SetVectorOption("diffuse", 0.302f, 0.876f, 0.727f);
        object_option.SetVectorOption("specular", 0.376f, 0.093f, 0.677f);
        MovingMeshAnimator animator;
        //viewer.AddStaticObject(vertices, elements, object_option);
        viewer.AddDynamicObject(vertices, elements, &animator, object_option);


        // A point light.
        opengl_viewer::Option light_option;
        light_option.SetVectorOption("ambient", 0.17f, 0.15f, 0.14f);
        light_option.SetVectorOption("diffuse", 0.37f, 0.35f, 0.34f);
        light_option.SetVectorOption("specular", 0.37f, 0.35f, 0.34f);
        viewer.AddStaticPointLight(Eigen::Vector3f(-2.0f, 1.0f, -2.0f), light_option);
        viewer.AddStaticPointLight(Eigen::Vector3f(2.0f, 1.0f, -2.0f), light_option);
        viewer.AddStaticPointLight(Eigen::Vector3f(-2.0f, 1.0f, 2.0f), light_option);
        viewer.AddStaticPointLight(Eigen::Vector3f(2.0f, 1.0f, 2.0f), light_option);

        MovingMeshLightAnimator light_animator;

        viewer.AddDynamicPointLight(&light_animator, light_option);


        viewer.Run();
        viewer.Cleanup();
    }

}