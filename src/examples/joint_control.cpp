#include <franka_control/Franka.hpp>

#include <control_lib/controllers/Feedback.hpp>
#include <control_lib/controllers/LinearDynamics.hpp>
#include <control_lib/spatial/RN.hpp>

using namespace franka_control;
using namespace control_lib;

struct Params {
    struct controller : public defaults::controller {
        // Integration time step controller
        PARAM_SCALAR(double, dt, 0.01);
    };

    struct feedback : public defaults::feedback {
        // Output dimension
        PARAM_SCALAR(size_t, d, 7);
    };
};

class ConfigController : public control::JointControl {
public:
    ConfigController() : control::JointControl(ControlMode::CONFIGURATIONSPACE)
    {
        // step
        _dt = 0.01;

        // gains
        Eigen::MatrixXd K = 10 * Eigen::MatrixXd::Identity(7, 7), D = 1 * Eigen::MatrixXd::Identity(7, 7);

        // goal
        spatial::RN<7> ref((Eigen::Matrix<double, 7, 1>() << 0.300325, 0.596986, 0.140127, -1.44853, 0.15547, 2.31046, 0.690596).finished());
        ref._vel = Eigen::Matrix<double, 7, 1>::Zero();

        // set controller
        _controller.setStiffness(K).setDamping(D).setReference(ref);
    }

    Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) override
    {
        // current state
        spatial::RN<7> curr(jointPosition(state));
        curr._vel = jointVelocity(state);

        // auto tau = _controller.action(curr);

        // std::cout << tau.transpose() << std::endl;

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

    // franka::RobotState state = robot.robot().readOnce();

    // auto ctr = std::make_unique<ConfigController>();

    // std::cout << ctr->action(state) << std::endl;

    // std::cout << ctr->jointPosition(state).transpose() << std::endl;

    robot.setJointController(std::make_unique<ConfigController>());

    robot.torque();

    return 0;
}
