#include "NNController.h"
#include "NNModelParameters.h"

namespace copter_simulation {

typedef Eigen::Map<Eigen::Matrix<float, OB_SPACE_SIZE, 1>> VectorObf;
typedef Eigen::Map<Eigen::Matrix<float, HIDDEN_LAYER_SIZE, 1>> VectorHiddenf;
typedef Eigen::Map<Eigen::Matrix<float, AC_SPACE_SIZE, 1>> VectorActf;

typedef Eigen::Map<Eigen::Matrix<float, OB_SPACE_SIZE, HIDDEN_LAYER_SIZE>, 0, Eigen::Stride<1, HIDDEN_LAYER_SIZE>> MatrixObHiddenf;
typedef Eigen::Map<Eigen::Matrix<float, HIDDEN_LAYER_SIZE, HIDDEN_LAYER_SIZE>, 0, Eigen::Stride<1, HIDDEN_LAYER_SIZE>> MatrixHiddenf;
typedef Eigen::Map<Eigen::Matrix<float, HIDDEN_LAYER_SIZE, AC_SPACE_SIZE>, 0, Eigen::Stride<1, AC_SPACE_SIZE>> MatrixHiddenActf;

NNController::NNController(InertialSensor& sensor, HybridCopter& copter):
    HighLevelController(sensor, copter) {
    _I_dt = 0.0025;
    W_hidden.clear();
    b_hidden.clear();
}

void NNController::reset() {
    // load running mean and std
    ob_mean = VectorObf(const_cast<float *>(copter_simulation::ob_mean));
    ob_std = VectorObf(const_cast<float *>(copter_simulation::ob_std));

    // load W's and b's
    W_hidden.push_back(MatrixObHiddenf(const_cast<float *>(&copter_simulation::W0[0][0])).transpose());
    b_hidden.push_back(VectorHiddenf(const_cast<float *>(copter_simulation::b0)));

    for (int i = 0; i < NUM_HIDDEN_LAYER - 1; i++)
        W_hidden.push_back(MatrixHiddenf(const_cast<float *>(&copter_simulation::W_hidden[i][0][0])).transpose());
    for (int i = 0; i < NUM_HIDDEN_LAYER - 1; i++)
        b_hidden.push_back(VectorHiddenf(const_cast<float *>(copter_simulation::b_hidden[i])));
    
    W_output = MatrixHiddenActf(const_cast<float *>(&copter_simulation::W1[0][0])).transpose();
    b_output = VectorActf(const_cast<float *>(copter_simulation::b1));

    // load output bias
    output_bias = Eigen::VectorXf::Constant(AC_SPACE_SIZE, FINAL_BIAS);

    // reset I_error
    _I_error = Eigen::Vector4f::Zero();
    _I_activated = false;
}

void NNController::output(const Eigen::VectorXf& rpyt_in, Eigen::VectorXf& rpyt_out) {
    // get target
    Eigen::Vector4f target = Eigen::Vector4f(rpyt_in(0), rpyt_in(1), rpyt_in(2), rpyt_in(3));

    // Create observation vector
    Eigen::Matrix<float, 9, 1> state = _sensor.delayed_state();

    Eigen::Vector3f rpy = state.segment(0, 3);
    Eigen::Vector3f vel = state.segment(3, 3);
    Eigen::Vector3f omega = state.segment(6, 3);
    Eigen::Vector3f vel_local = Eigen::AngleAxisf(-rpy(2), Eigen::Vector3f::UnitZ()).matrix() * vel;
    Eigen::Vector4f error = target - Eigen::Vector4f(vel_local(0), vel_local(1), vel_local(2), rpy(2));
    error[3] = copter_utils::wrap2PI(error[3]);

    if (rpyt_in(2) < 0.0) {
        _I_activated = true;
    }
    _I_activated = true;
    if (_I_activated) {
        _I_error = _I_error * 0.997 + _I_dt * error;
        for (int i = 0;i < _I_error.size();i++) {
            if (_I_error[i] < -5.0) {
                _I_error[i] = -5.0;
            }
            if (_I_error[i] > 5.0) {
                _I_error[i] = 5.0;
            }
        }
    }

    Eigen::VectorXf ob(OB_SPACE_SIZE);
    ob.setZero();
    ob.head(3) = rpy;
    ob.segment(3, 3) = omega;
    ob.segment(6, 3) = error.head(3);
    ob.segment(9, 4) = _I_error;
    // ob.segment(3, 3) = vel_local;
    // ob.segment(6, 3) = omega;
    // ob(9) = target[0];
    // ob[10] = target[2];

    float target_yaw_diff = target(3);
    ob(2) -= target_yaw_diff;
    ob(2) = copter_utils::wrap2PI(ob(2));
    
    // NN Computation
    // std::cout << "NN Computation start" << std::endl;
    ob = (ob - ob_mean).cwiseQuotient(ob_std);
    ob = ob.unaryExpr([](float x) {
        return std::min(std::max(x, -5.0f), 5.0f);
    });
    // std::cerr << "final ob = " << ob.transpose() << std::endl;
    Eigen::VectorXf last_out = ob;
    for (int i = 0; i < NUM_HIDDEN_LAYER; i++) {
        last_out = (W_hidden[i] * last_out + b_hidden[i]).array().tanh();
    }
    rpyt_out = (W_output * last_out + b_output + output_bias).cast<float>();

    for (int i = 0;i < rpyt_out.size();++i) {
        if (rpyt_out[i] <= 0.0) {
            rpyt_out[i] = 0.0;
        }
        if (rpyt_out[i] >= output_bias[i] * 2.0) {
            rpyt_out[i] = output_bias[i] * 2.0;
        }
    }

    // std::cout << "NN Computation end" << std::endl;
    // std::cerr << "motor out = " << rpyt_out.transpose() << std::endl;
    // std::cout << "[" << rpyt_out.transpose().eval().format(fmt_prec) << "]" << std::endl;
}

int NNController::action_space_dim() { return AC_SPACE_SIZE; }

int NNController::ob_space_dim() { return OB_SPACE_SIZE; }

}
