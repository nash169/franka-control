#ifndef FRANKACONTROL_FRANKA_HPP
#define FRANKACONTROL_FRANKA_HPP

#include <Eigen/Core>
#include <chrono>
#include <franka/model.h>
#include <franka/robot.h>
#include <future>
#include <iostream>
#include <memory>
#include <thread>
#include <utils_lib/FileManager.hpp>

#include "franka_control/control/JointControl.hpp"
#include "franka_control/control/TaskControl.hpp"
#include "franka_control/tools/common.hpp"

using namespace std::chrono_literals;

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

        Franka& setJointController(std::unique_ptr<control::JointControl> controller)
        {
            _joint_controller = std::move(controller);

            return *this;
        }

        Franka& setTaskController(std::unique_ptr<control::TaskControl> controller)
        {
            _task_controller = std::move(controller);

            return *this;
        }

        void torque(const size_t& delay = 1)
        {
            Eigen::Matrix<double, 7, 1> tau = Eigen::Matrix<double, 7, 1>::Zero();

            auto start = [&]() -> Eigen::Matrix<double, 7, 1> {
                std::this_thread::sleep_for(std::chrono::seconds(delay));
                std::cout << "Start Torque Control" << std::endl;
                return Eigen::Matrix<double, 7, 1>::Zero();
            };

            auto task = std::async(std::launch::async, start);

            auto controller = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                if (task.wait_for(0ms) == std::future_status::ready) {
                    tau = task.get();
                    task = std::async(std::launch::async, &control::JointControl::action, _joint_controller.get(), robot_state);
                }

                return {{tau[0], tau[1], tau[2], tau[3], tau[4], tau[5], tau[6]}};
            };

            try {
                _robot->control(controller);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

        void jointPosition(const size_t& delay = 1)
        {
            Eigen::Matrix<double, 7, 1> pos = Eigen::Matrix<double, 7, 1>::Zero();

            auto start = [&]() -> Eigen::Matrix<double, 7, 1> {
                std::this_thread::sleep_for(std::chrono::seconds(delay));
                std::cout << "Start Position Control" << std::endl;
                return Eigen::Matrix<double, 7, 1>::Zero();
            };

            auto task = std::async(std::launch::async, start);

            auto controller = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
                if (task.wait_for(0ms) == std::future_status::ready) {
                    pos = task.get();
                    task = std::async(std::launch::async, &control::JointControl::action, _joint_controller.get(), robot_state);
                }

                return {{pos[0], pos[1], pos[2], pos[3], pos[4], pos[5], pos[6]}};
            };

            try {
                _robot->control(controller);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

        void jointVelocity(const size_t& delay = 1)
        {
            Eigen::Matrix<double, 7, 1> vel = Eigen::Matrix<double, 7, 1>::Zero();

            auto start = [&]() -> Eigen::Matrix<double, 7, 1> {
                std::this_thread::sleep_for(std::chrono::seconds(delay));
                std::cout << "Start Velocity Control" << std::endl;
                return Eigen::Matrix<double, 7, 1>::Zero();
            };

            auto task = std::async(std::launch::async, start);

            auto controller = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::JointPositions {
                if (task.wait_for(0ms) == std::future_status::ready) {
                    vel = task.get();
                    task = std::async(std::launch::async, &control::JointControl::action, _joint_controller.get(), robot_state);
                }

                return {{vel[0], vel[1], vel[2], vel[3], vel[4], vel[5], vel[6]}};
            };

            try {
                _robot->control(controller);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

        void record(const std::string& path, const size_t& frequency = 100, const size_t& delay = 1)
        {
            utils_lib::FileManager recorder(path);

            auto start = [&]() {
                std::this_thread::sleep_for(std::chrono::seconds(delay));
                std::cout << "Start Recording" << std::endl;
            };

            auto task = std::async(std::launch::async, start);

            auto record = [&](const franka::RobotState& robot_state, franka::Duration period) {
                // End Effector
                Eigen::Map<const Eigen::Matrix<double, 1, 16>> ee(robot_state.O_T_EE.data());

                recorder.append(ee);
            };

            size_t time = 0;

            auto control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                if (task.wait_for(0ms) == std::future_status::ready && (time * frequency) % 1000 == 0)
                    task = std::async(std::launch::async, record, robot_state, period);

                time += period.toMSec();

                return {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
            };

            try {
                _robot->control(control_callback);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

    protected:
        // Robot Handle
        std::unique_ptr<franka::Robot> _robot;

        // Model
        std::unique_ptr<franka::Model> _model;

        // Controllers
        std::unique_ptr<control::JointControl> _joint_controller;
        std::unique_ptr<control::TaskControl> _task_controller;
    };
} // namespace franka_control

#endif // FRANKACONTROL_FRANKA_HPP