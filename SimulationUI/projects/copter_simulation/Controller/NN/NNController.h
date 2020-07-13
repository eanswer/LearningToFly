#pragma once
#include "Sensor/InertialSensor.h"
#include "Controller/HighLevelController.h"

namespace copter_simulation {

class NNController : public HighLevelController {
public:
    NNController(InertialSensor& sensor, HybridCopter& copter);

    // load parameters from the header file generated elsewhere
    void reset();

    // input is target roll, target pitch, target yaw rate, target climb rate
    void output(const Eigen::VectorXf& rpyt_in, Eigen::VectorXf& rpyt_out);

    int action_space_dim();
    int ob_space_dim();

private:
    // structure of NN
    Eigen::VectorXf ob_mean, ob_std;
    std::vector<Eigen::MatrixXf> W_hidden;
    std::vector<Eigen::VectorXf> b_hidden;
    Eigen::MatrixXf W_output;
    Eigen::VectorXf b_output;
    Eigen::VectorXf output_bias;

    // NN variables
    Eigen::Vector4f _I_error;
    float _I_dt;
    bool _I_activated;
};

}
