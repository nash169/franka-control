#include <iostream>

// Robot Handle
#include <franka_control/Franka.hpp>

// Task Space Manifolds
#include <control_lib/spatial/R.hpp>
#include <control_lib/spatial/SO.hpp>

// Robot Model
#include <beautiful_bullet/bodies/MultiBody.hpp>

// Task Space Dynamical System & Derivative Controller
#include <control_lib/controllers/Feedback.hpp>

using namespace franka_control;
using namespace control_lib;
using namespace beautiful_bullet;

using R3 = spatial::R<3>;
using SO3 = spatial::SO<3, true>;

struct Params {
    struct controller : public defaults::controller {
        // Integration time step controller
        PARAM_SCALAR(double, dt, 0.01);
    };

    struct feedback : public defaults::feedback {
        // Output dimension
        PARAM_SCALAR(size_t, d, 3);
    };
};

struct FrankaModel : public bodies::MultiBody {
public:
    FrankaModel() : bodies::MultiBody("models/franka/urdf/panda.urdf"), _frame("panda_joint_7"), _reference(pinocchio::LOCAL_WORLD_ALIGNED) {}

    Eigen::MatrixXd jacobian(const Eigen::VectorXd& q)
    {
        return static_cast<bodies::MultiBody*>(this)->jacobian(q, _frame, _reference);
    }

    Eigen::MatrixXd jacobianDerivative(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
    {
        return static_cast<bodies::MultiBody*>(this)->jacobianDerivative(q, dq, _frame, _reference);
    }

    Eigen::Matrix<double, 6, 1> framePose(const Eigen::VectorXd& q)
    {
        return static_cast<bodies::MultiBody*>(this)->framePose(q, _frame);
    }

    Eigen::Matrix<double, 6, 1> frameVelocity(const Eigen::VectorXd& q, const Eigen::VectorXd& dq)
    {
        return static_cast<bodies::MultiBody*>(this)->frameVelocity(q, dq, _frame, _reference);
    }

    std::string _frame;
    pinocchio::ReferenceFrame _reference;
};

class R3SO3Controller : public franka_control::control::JointControl {
public:
    R3SO3Controller() : franka_control::control::JointControl()
    {
        // reference
        Eigen::Vector3d position(0.38276186, -0.67454143, 0.10267061);
        Eigen::Matrix3d orientation;
        orientation << 0.922046, 0.377679, 0.0846751,
            0.34527, -0.901452, 0.261066,
            0.17493, -0.211479, -0.9616;
        _r3_ref = R3(position);
        _so3_ref = SO3(orientation);

        // ds
        _r3_ds.setStiffness(5.0 * Eigen::MatrixXd::Identity(3, 3))
            .setReference(_r3_ref);
        _so3_ds.setStiffness(5.0 * Eigen::MatrixXd::Identity(3, 3))
            .setReference(_so3_ref);

        // ctr
        _r3_ctr.setDamping(5.0 * Eigen::MatrixXd::Identity(3, 3));
        _so3_ctr.setDamping(5.0 * Eigen::MatrixXd::Identity(3, 3));

        // model
        _model = std::make_shared<FrankaModel>();
    }

    Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) override
    {
        // state
        Eigen::Matrix<double, 7, 1> q = jointPosition(state), dq = jointVelocity(state);
        Eigen::Matrix<double, 6, 1> pose = _model->framePose(q);
        R3 _r3_curr(_model->framePosition(q));
        SO3 _so3_curr(_model->frameOrientation(q));
        Eigen::Matrix<double, 6, 7> jac = _model->jacobian(q);
        Eigen::Matrix<double, 6, 1> pose_v = jac * dq;
        _r3_curr._v = pose_v.head(3);
        _so3_curr._v = pose_v.tail(3);
        _r3_ref._v = _r3_ds(_r3_curr);
        _so3_ref._v = _so3_ds(_so3_curr);

        // // current task space state
        // auto pose = taskPose(state);
        // R3 _r3_curr(pose.translation());
        // SO3 _so3_curr(pose.linear());

        // Eigen::Matrix<double, 6, 7> jac = jacobian(state);
        // Eigen::Matrix<double, 6, 1> vel = jac * jointVelocity(state);
        // _r3_curr._v = vel.head(3);
        // _so3_curr._v = vel.tail(3);

        // // ds
        // _r3_ref._v = _r3_ds(_r3_curr);
        // _so3_ref._v = _so3_ds(_so3_curr);

        // ctr
        // return Eigen::Matrix<double, 7, 1>::Zero();
        return jac.transpose() * (Eigen::Matrix<double, 6, 1>() << _r3_ctr.setReference(_r3_ref).action(_r3_curr), _so3_ctr.setReference(_so3_ref).action(_so3_curr)).finished();
    }

protected:
    // reference state
    R3 _r3_ref;
    SO3 _so3_ref;

    // task space position ds
    controllers::Feedback<Params, R3> _r3_ds;
    // task space orientation ds
    controllers::Feedback<Params, SO3> _so3_ds;
    // task space position controller
    controllers::Feedback<Params, R3> _r3_ctr;
    // task space orientation controller
    controllers::Feedback<Params, SO3> _so3_ctr;
    // model
    std::shared_ptr<FrankaModel> _model;
};

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    robot.setJointController(std::make_unique<R3SO3Controller>());

    robot.torque();

    return 0;
}
