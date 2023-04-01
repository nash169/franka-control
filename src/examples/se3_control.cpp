#include <iostream>

// Robot Handle
#include <franka_control/Franka.hpp>

// Task Space Manifolds
#include <control_lib/spatial/SE.hpp>

// Robot Model
#include <beautiful_bullet/bodies/MultiBody.hpp>

// Task Space Dynamical System
#include <control_lib/controllers/LinearDynamics.hpp>

// Task Space Derivative Controller
#include <control_lib/controllers/Feedback.hpp>

using namespace franka_control;
using namespace control_lib;
using namespace beautiful_bullet;

struct Params {
    struct controller : public defaults::controller {
        // Integration time step controller
        PARAM_SCALAR(double, dt, 0.01);
    };

    struct feedback : public defaults::feedback {
        // Output dimension
        PARAM_SCALAR(size_t, d, 6);
    };

    struct linear_dynamics : public defaults::linear_dynamics {
    };
};

class SE3Controller : public franka_control::control::JointControl {
public:
    SE3Controller() : franka_control::control::JointControl()
    {
        // reference
        Eigen::Vector3d position(0.683783, 0.308249, 0.185577);
        Eigen::Matrix3d orientation;
        orientation << 0.922046, 0.377679, 0.0846751,
            0.34527, -0.901452, 0.261066,
            0.17493, -0.211479, -0.9616;

        _se3_ref = spatial::SE<3>(orientation, position);

        // ds
        Eigen::MatrixXd A = 1.0 * Eigen::MatrixXd::Identity(3, 3);
        _se3_ds.setDynamicsMatrix(A);
        _se3_ds.setReference(_se3_ref);

        // feedback
        Eigen::MatrixXd D = 1.0 * Eigen::MatrixXd::Identity(3, 3);
        _se3_feedback.setDamping(D);

        // model
        _model = std::make_shared<bodies::MultiBody>("models/franka/urdf/panda.urdf");
    }

    Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) override
    {
        // current task space state (ds input)
        auto pose = taskPose(state);
        spatial::SE<3> _se3_curr(pose.rotation(), pose.translation());

        auto jac = jacobian(state);
        _se3_curr._v = jac * jointVelocity(state);

        // ds
        _se3_ref._v = _se3_ds.action(_se3_curr);

        return jac.transpose() * _se3_feedback.setReference(_se3_ref).action(_se3_curr);
    }

protected:
    // reference state
    spatial::SE<3> _se3_ref;

    // task space ds
    controllers::LinearDynamics<Params, spatial::SE<3>> _se3_ds;

    // task space controller (in this case this space is actually R3 x SO3)
    controllers::Feedback<Params, spatial::SE<3>> _se3_feedback;

    // Model
    bodies::MultiBodyPtr _model;
};

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    robot.setJointController(std::make_unique<SE3Controller>());

    robot.torque();

    return 0;
}
