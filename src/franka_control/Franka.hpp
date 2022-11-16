#ifndef FRANKACONTROL_FRANKA_HPP
#define FRANKACONTROL_FRANKA_HPP

#include <Eigen/Core>
#include <franka/model.h>
#include <franka/robot.h>
#include <iostream>
#include <memory>
#include <utils_lib/FileManager.hpp>

#include "franka_control/tools/common.hpp"

namespace franka_control {
    class Franka {
    public:
        Franka(const std::string& franka_address)
        {
            try {
                // connect to robot
                _robot = std::make_unique<franka::Robot>(franka_address);

                // set default beahvior
                tools::setDefaultBehavior(*_robot);

                // load the kinematics and dynamics model
                franka::Model model = _robot->loadModel();
            }
            catch (const std::exception& ex) {
                // print exception
                std::cout << ex.what() << std::endl;
            }
        }

        void record(const std::string& path)
        {
            utils_lib::FileManager recorder(path);

            auto control_callback = [&](const franka::RobotState& robot_state, franka::Duration) -> franka::Torques {
                // // Configuration Space
                // Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data()),
                //     dq(robot_state.dq.data()),
                //     tau(robot_state.tau_J.data());

                // // End Effector
                // Eigen::Map<const Eigen::Matrix<double, 1, 16>> ee(robot_state.O_T_EE.data());

                // recorder.append(ee);

                return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            };

            try {
                // // start real-time control loop
                // _robot->control(control_callback);
                size_t count = 0;
                _robot->read([&count](const franka::RobotState& robot_state) {
                    // Printing to std::cout adds a delay. This is acceptable for a read loop such as this,
                    // but should not be done in a control loop.
                    Eigen::Map<const Eigen::Matrix<double, 1, 16>> ee(robot_state.O_T_EE.data());
                    std::cout << ee << std::endl;
                    return true;
                });
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

    protected:
        std::unique_ptr<franka::Robot> _robot;
        std::unique_ptr<franka::Model> _model;
    };
} // namespace franka_control

#endif // FRANKACONTROL_FRANKA_HPP