#include <iostream>

// Robot Handle
#include <franka_control/Franka.hpp>

using namespace franka_control;

class TaskController : public control::JointControl {
public:
    // TaskController() : control::JointControl(ControlMode::OPERATIONSPACE)
    TaskController() : control::JointControl()
    {
        // ref
        _xRef << 0.683783, 0.308249, 0.185577;
        _oRef << 0.922046, 0.377679, 0.0846751,
            0.34527, -0.901452, 0.261066,
            0.17493, -0.211479, -0.9616;
    }

    Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) override
    {
        // current state (ds input)
        auto pose = taskPose(state);

        Eigen::Matrix<double, 6, 1> error;
        error.head(3) << pose.translation() - _xRef;

        // orientation error
        Eigen::Quaterniond orientation(pose.linear()), orientation_d(_oRef);
        // "difference" quaternion
        if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
            orientation.coeffs() << -orientation.coeffs();
        }
        // "difference" quaternion
        Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
        error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
        // Transform to base frame
        error.tail(3) << -pose.linear() * error.tail(3);

        // DS
        Eigen::MatrixXd A = 10 * Eigen::MatrixXd::Identity(6, 6);
        auto x_dot = -A * error;

        // jacobian
        auto jac = jacobian(state);

        // Controller
        Eigen::MatrixXd D = 5 * Eigen::MatrixXd::Identity(6, 6);
        auto force = -D * (jac * jointVelocity(state) - x_dot);

        return jac.transpose() * force;
    }

protected:
    // desired final state
    Eigen::Vector3d _xRef;
    Eigen::Matrix3d _oRef;
};

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    // Eigen::Vector3d xDes(0.683783, 0.308249, 0.185577);
    // Eigen::Matrix3d oDes;
    // oDes << 0.922046, 0.377679, 0.0846751,
    //     0.34527, -0.901452, 0.261066,
    //     0.17493, -0.211479, -0.9616;

    // Eigen::Vector3d position_d(xDes);
    // Eigen::Quaterniond orientation_d(oDes);

    // franka::RobotState state = robot.robot().readOnce();

    // Eigen::Affine3d transform(Eigen::Matrix4d::Map(state.O_T_EE.data()));
    // Eigen::Vector3d position(transform.translation());
    // Eigen::Quaterniond orientation(transform.linear());

    // Eigen::Matrix<double, 6, 1> error;
    // error.head(3) << position - position_d;

    // // orientation error
    // // "difference" quaternion
    // if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
    //     orientation.coeffs() << -orientation.coeffs();
    // }
    // // "difference" quaternion
    // Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    // error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // // Transform to base frame
    // error.tail(3) << -transform.linear() * error.tail(3);

    // Eigen::AngleAxisd aa(oDes * transform.linear().transpose());
    // std::cout << aa.angle() * aa.axis() << std::endl;
    // std::cout << "--" << std::endl;
    // std::cout << error.tail(3) << std::endl;

    // auto ctr = std::make_unique<TaskController>();

    // std::cout << ctr->taskPose(state).linear() << std::endl;
    // std::cout << "--" << std::endl;
    // std::cout << ctr->taskPose(state).translation().transpose() << std::endl;

    // std::cout << ctr->action(state) << std::endl;

    // std::cout << ctr->jointPosition(state).transpose() << std::endl;

    robot.setJointController(std::make_unique<TaskController>());

    // robot.move();
    robot.torque();

    return 0;
}
