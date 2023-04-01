#include <iostream>

// Robot Handle
#include <franka_control/Franka.hpp>

// Optitrack Handle
// #include <optitrack_lib/Optitrack.hpp>

// Task Space Manifolds
#include <control_lib/spatial/RN.hpp>
#include <control_lib/spatial/SE3.hpp> // to test using the jacobian from Pinocchio
#include <control_lib/spatial/SO3.hpp>

// Task Space Dynamical System
#include <control_lib/controllers/LinearDynamics.hpp>

// Task Space Derivative Controller
#include <control_lib/controllers/Feedback.hpp>

using namespace franka_control;
using namespace control_lib;
// using namespace optitrack_lib;

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

class ExternalController : public control::JointControl {
public:
    ExternalController() : control::JointControl()
    {
        // step
        _dt = 0.01;

        // translation feedback
        Eigen::MatrixXd Dt = 1.0 * Eigen::MatrixXd::Identity(3, 3);
        _translation_feedback.setDamping(Dt);

        // rotation feedback
        Eigen::MatrixXd Dr = 1.0 * Eigen::MatrixXd::Identity(3, 3);
        _rotation_feedback.setDamping(Dr);

        // translation ds
        Eigen::MatrixXd At = 5.0 * Eigen::MatrixXd::Identity(3, 3);
        _translation_ds.setDynamicsMatrix(At);
        Eigen::Vector3d position(0.683783, 0.308249, 0.185577);
        _xRef._pos = position;
        _translation_ds.setReference(_xRef);

        // rotation ds
        Eigen::MatrixXd Ar = 1.0 * Eigen::MatrixXd::Identity(3, 3);
        _rotation_ds.setDynamicsMatrix(Ar);
        Eigen::Matrix3d orientation;
        orientation << 0.922046, 0.377679, 0.0846751,
            0.34527, -0.901452, 0.261066,
            0.17493, -0.211479, -0.9616;
        _oRef._rot = orientation;
        _rotation_ds.setReference(_oRef);
    }

    Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) override
    {
        // current task space state (ds input)
        auto pose = taskPose(state);
        spatial::RN<3> xCurr(pose.translation());
        spatial::SO3 oCurr(pose.rotation());

        auto jac = jacobian(state);
        auto vel = jac * jointVelocity(state);

        xCurr._vel = vel.head(3);
        oCurr._vel = vel.tail(3);

        // translation ds
        _xRef._vel = _translation_ds.action(xCurr);

        // rotation ds
        _oRef._vel = _rotation_ds.action(oCurr);

        auto tau = jac.transpose() * (Eigen::Matrix<double, 6, 1>() << _translation_feedback.setReference(_xRef).action(xCurr), _rotation_feedback.setReference(_oRef).action(oCurr)).finished();
        // auto tau = jac.transpose() * (Eigen::Matrix<double, 6, 1>() << _translation_feedback.setReference(_xRef).action(xCurr), Eigen::Vector3d::Zero()).finished();
        // std::cout << tau.transpose() << std::endl;

        return tau;
    }

protected:
    // step
    double _dt;

    // reference state
    spatial::RN<3> _xRef;
    spatial::SO3 _oRef;

    // task space ds
    controllers::LinearDynamics<Params, spatial::RN<3>> _translation_ds;
    controllers::LinearDynamics<Params, spatial::SO3> _rotation_ds;

    // task space controller (in this case this space is actually R3 x SO3)
    controllers::Feedback<Params, spatial::RN<3>> _translation_feedback;
    controllers::Feedback<Params, spatial::SO3> _rotation_feedback;
};

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    robot.setJointController(std::make_unique<ExternalController>());

    robot.torque();

    return 0;
}