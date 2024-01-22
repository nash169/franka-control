#ifndef FRANKACONTROL_FRANKA_HPP
#define FRANKACONTROL_FRANKA_HPP

#include <Eigen/Core>
#include <chrono>
#include <franka/model.h>
#include <franka/robot.h>
#include <future>
#include <iostream>
#include <memory>
#include <shared_mutex>
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
                _model = std::make_shared<franka::Model>(_robot->loadModel());
            }
            catch (const std::exception& ex) {
                // print exception
                std::cout << ex.what() << std::endl;
            }
        }

        franka::Robot& robot()
        {
            return *_robot;
        }

        Franka& setJointController(std::unique_ptr<control::JointControl> controller)
        {
            _joint_controller = std::move(controller);
            _joint_controller->setModel(_model);

            return *this;
        }

        Franka& setTaskController(std::unique_ptr<control::TaskControl> controller)
        {
            _task_controller = std::move(controller);
            _task_controller->setModel(_model);

            return *this;
        }

        void move()
        {
            auto controller = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                return {{0, 0, 0, 0, 0, 0, 0}};
            };

            try {
                _robot->control(controller);
            }
            catch (const std::exception& e) {
                std::cerr << e.what() << '\n';
            }
        }

        void torque(const size_t& delay = 1)
        {
            // Eigen::Matrix<double, 7, 1> tau = Eigen::Matrix<double, 7, 1>::Zero();
            _control.setZero();
            std::atomic<bool> stop = false;

            // auto init = [&]() -> Eigen::Matrix<double, 7, 1> {
            //     // std::this_thread::sleep_for(std::chrono::seconds(delay));
            //     // std::cout << "Start Torque Control" << std::endl;
            //     return Eigen::Matrix<double, 7, 1>::Zero();
            // };

            auto control = [&]() {
                while (!stop) {
                    // Read state
                    std::unique_lock<std::shared_mutex> guard(_mutex);
                    // franka::RobotState state = _state;
                    guard.unlock();

                    // Compute controller
                    Eigen::Matrix<double, 7, 1> control = _joint_controller->action(_state);

                    // Write controller
                    guard.lock();
                    _control = control;
                }
            };

            std::thread control_thread(control);

            auto controller = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                std::shared_lock<std::shared_mutex> guard(_mutex);
                _state = robot_state;
                return {{_control[0], _control[1], _control[2], _control[3], _control[4], _control[5], _control[6]}};
            };

            try {
                _robot->control(controller);
                stop = true;
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
                // recorder.write("joint position - joint velocity - ee pose -- ee velocity - torque");
            };

            auto task = std::async(std::launch::async, start);

            auto record = [&](const franka::RobotState& state) {
                Eigen::Matrix<double, 1, 40> record_vec;

                // Joint position
                record_vec.head(7) = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());

                // Joint velocity
                record_vec.segment(7, 7) = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data());

                // End-effector pose
                Eigen::Affine3d pose(Eigen::Matrix4d::Map(state.O_T_EE.data()));
                Eigen::AngleAxisd aa(pose.linear());
                record_vec.segment(14, 6) << pose.translation().transpose(), aa.angle() * aa.axis().transpose();

                // End-effector velocity
                record_vec.segment(20, 6) = Eigen::Map<const Eigen::Matrix<double, 6, 7>>(_model->zeroJacobian(franka::Frame::kEndEffector, state).data()) * record_vec.segment(7, 7).transpose();

                // Torques
                record_vec.segment(26, 7) = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.tau_J.data());

                // Gravity
                record_vec.tail(7) = Eigen::Map<const Eigen::Matrix<double, 7, 1>>(_model->gravity(state).data());

                recorder.append(record_vec);
            };

            size_t time = 0;

            auto control_callback = [&](const franka::RobotState& robot_state, franka::Duration period) -> franka::Torques {
                if (task.wait_for(0ms) == std::future_status::ready && (time * frequency) % 1000 == 0)
                    task = std::async(std::launch::async, record, robot_state);

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
        std::shared_ptr<franka::Model> _model;

        // State & Control
        franka::RobotState _state;
        Eigen::Matrix<double, 7, 1> _control;

        // Controllers
        std::unique_ptr<control::JointControl> _joint_controller;
        std::unique_ptr<control::TaskControl> _task_controller;

        std::shared_mutex _mutex;
    };
} // namespace franka_control

#endif // FRANKACONTROL_FRANKA_HPP