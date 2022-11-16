#include <array>
#include <iostream>

#include <Eigen/Core>

#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>

#include <franka_control/tools/common.hpp>

using namespace franka_control;

int main(int argc, char** argv)
{
    // Check whether the required arguments were passed
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
        return -1;
    }
    // parameters
    double desired_mass{0.0};
    constexpr double target_mass{1.0}; // NOLINT(readability-identifier-naming)
    constexpr double k_p{1.0}; // NOLINT(readability-identifier-naming)
    constexpr double k_i{2.0}; // NOLINT(readability-identifier-naming)
    constexpr double filter_gain{0.001}; // NOLINT(readability-identifier-naming)

    try {
        // connect to robot
        franka::Robot robot(argv[1]);
        tools::setDefaultBehavior(robot);
        // load the kinematics and dynamics model
        franka::Model model = robot.loadModel();

        // set collision behavior
        robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
            {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

        franka::RobotState initial_state = robot.readOnce();

        Eigen::VectorXd initial_tau_ext(7), tau_error_integral(7);
        // Bias torque sensor
        std::array<double, 7> gravity_array = model.gravity(initial_state);
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_tau_measured(initial_state.tau_J.data());
        Eigen::Map<Eigen::Matrix<double, 7, 1>> initial_gravity(gravity_array.data());
        initial_tau_ext = initial_tau_measured - initial_gravity;

        // init integrator
        tau_error_integral.setZero();

        // define callback for the torque control loop
        Eigen::Vector3d initial_position;
        double time = 0.0;
        auto get_position = [](const franka::RobotState& robot_state) {
            auto copy_array = robot_state.O_T_EE;
            Eigen::VectorXd vec = Eigen::Map<Eigen::VectorXd>(copy_array.data(), copy_array.size());

            // std::cout << vec.transpose() << std::endl;

            Eigen::MatrixXd mat = Eigen::Map<Eigen::MatrixXd>(vec.data(), 4, 4);

            // std::cout << mat << std::endl;

            return Eigen::Vector3d(robot_state.O_T_EE[12], robot_state.O_T_EE[13],
                robot_state.O_T_EE[14]);
        };

        auto force_control_callback = [&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
            get_position(robot_state);
            return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
        };

        // start real-time control loop
        robot.control(force_control_callback);
    }
    catch (const std::exception& ex) {
        // print exception
        std::cout << ex.what() << std::endl;
    }
    return 0;
}
