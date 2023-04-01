#include <iostream>

// Robot Handle
#include <franka_control/Franka.hpp>

// Optitrack Handle
#include <optitrack_lib/Optitrack.hpp>

// ZMQ Stream
// #include <zmq_stream/Publisher.hpp>
// #include <zmq_stream/Subscriber.hpp>
#include <zmq_stream/Requester.hpp>

#include <beautiful_bullet/bodies/MultiBody.hpp>

#include <utils_lib/Timer.hpp>

using namespace franka_control;
using namespace optitrack_lib;
using namespace zmq_stream;
using namespace beautiful_bullet;

class TaskController : public franka_control::control::JointControl {
public:
    TaskController() : franka_control::control::JointControl()
    {
        // step
        _dt = 0.01;

        // ref
        _xRef << 0.683783, 0.308249, 0.185577;
        _oRef << 0.922046, 0.377679, 0.0846751,
            0.34527, -0.901452, 0.261066,
            0.17493, -0.211479, -0.9616;

        // _publisher.configure("0.0.0.0", "5511");
        // _subscriber.configure("128.178.145.171", "5510");
        // _requester.configure("128.178.145.171", "5511");

        franka = std::make_shared<bodies::MultiBody>("models/franka/urdf/panda.urdf");
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

        // std::cout << "quat1" << std::endl;
        // std::cout << error.tail(3).transpose() << std::endl;

        // DS
        Eigen::MatrixXd A = 10 * Eigen::MatrixXd::Identity(6, 6);
        auto x_dot = -A * error;

        // jacobian
        auto jac = jacobian(state);

        // Controller
        Eigen::MatrixXd D = 1 * Eigen::MatrixXd::Identity(6, 6);
        auto force = -D * (jac * jointVelocity(state) - x_dot);

        Eigen::Matrix<double, 7, 1> tau = jac.transpose() * force;

        auto aa1 = Eigen::AngleAxisd(error_quaternion);
        auto aa2 = Eigen::AngleAxisd(_oRef.transpose() * pose.linear());
        auto aa3 = Eigen::AngleAxisd(pose.linear() * _oRef.transpose());
        auto aa4 = Eigen::AngleAxisd(pose.linear().transpose() * _oRef);
        // std::cout << "quat2" << std::endl;
        // std::cout << sin(aa.angle() / 2) * aa.axis().transpose() << std::endl;

        std::cout << "quat1" << std::endl;
        std::cout << aa1.angle() * (pose.linear() * aa1.axis()).transpose() << std::endl;
        std::cout << "quat2" << std::endl;
        std::cout << aa2.angle() * aa2.axis().transpose() << std::endl;
        std::cout << "quat3" << std::endl;
        std::cout << aa3.angle() * aa3.axis().transpose() << std::endl;
        std::cout << "quat4" << std::endl;
        std::cout << aa4.angle() * aa4.axis().transpose() << std::endl;

        // auto tmp = Eigen::AngleAxisd(pose.linear() * omega_skew);
        // std::cout << tmp.angle() * tmp.axis().transpose() << std::endl;
        // std::cout << omega.transpose() << std::endl;
        // std::cout << "hello" << std::endl;
        // auto tmp = Eigen::Vector3d(error_quaternion.x(), error_quaternion.y(), error_quaternion.z());
        // std::cout << pose.linear() << std::endl;
        // std::cout << (pose.rotation() * tmp).transpose() << std::endl;

        // {
        // // utils_lib::Timer time;
        // auto jac_pinocchio1 = franka->jacobian(jointPosition(state), "panda_link7", pinocchio::LOCAL);
        // auto jac_pinocchio2 = franka->jacobian(jointPosition(state), "panda_link7", pinocchio::LOCAL_WORLD_ALIGNED);
        // auto jac_pinocchio3 = franka->jacobian(jointPosition(state), "panda_link7", pinocchio::WORLD);

        // // }
        // Eigen::Matrix<double, 6, 6> mat;
        // mat.setZero();
        // mat.block(0, 0, 3, 3) = pose.rotation();
        // mat.block(3, 3, 3, 3) = pose.rotation();
        // std::cout << "jac1" << std::endl;
        // std::cout << jac_pinocchio1 << std::endl;
        // std::cout << "jac2" << std::endl;
        // std::cout << jac_pinocchio2 << std::endl;
        // std::cout << "jac3" << std::endl;
        // std::cout << jac_pinocchio3 << std::endl;
        // std::cout << "jac4" << std::endl;
        // std::cout << jac << std::endl;
        // std::cout << "jac5" << std::endl;
        // std::cout << mat * jac_pinocchio1 << std::endl;

        tau.setZero();

        return tau;
    }

protected:
    // step
    double _dt;

    // desired final state
    Eigen::Vector3d _xRef;
    Eigen::Matrix3d _oRef;

    // // optitrack
    // Optitrack _optitrack;

    // stream
    // Publisher _publisher;
    // Subscriber _subscriber;
    Requester _requester;

    bodies::MultiBodyPtr franka;
};

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    robot.setJointController(std::make_unique<TaskController>());

    // robot.move();
    robot.torque();

    return 0;
}
