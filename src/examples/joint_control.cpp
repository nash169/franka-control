#include <franka_control/Franka.hpp>

#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/LinearDynamics.hpp>
#include <control_lib/spatial/RN.hpp>

using namespace franka_control;
using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
    };

    struct feedback : public defaults::feedback {
    };

    struct linear_dynamics : public defaults::linear_dynamics {
    };
};

class ConfigController : public control::JointControl {
public:
    ConfigController() : control::JointControl(ControlMode::CONFIGURATIONSPACE)
    {
        // step
        _dt = 0.01;

        // gains
        Eigen::MatrixXd K = 1 * Eigen::MatrixXd::Identity(7, 7), D = 0.1 * Eigen::MatrixXd::Identity(7, 7);

        // goal
        spatial::RN<7> ref((Eigen::Matrix<double, 7, 1>() << 0.365308, -0.0810892, 1.13717, 0.365308, -0.0810892, 1.13717).finished());
        ref._vel = Eigen::Matrix<double, 7, 1>::Zero();

        // set controller
        _controller.setStiffness(K).setDamping(D).setReference(ref);
    }

    Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) override
    {
        // current state
        spatial::RN<7> curr(jointPosition(state));
        curr._vel = jointVelocity(state);

        return _controller.action(curr);
    }

protected:
    // step
    double _dt;

    // controller
    controllers::Feedback<Params, spatial::RN<7>> _controller;
};

int main(int argc, char const* argv[])
{
    Franka robot("franka");
    return 0;
}
