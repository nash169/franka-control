#ifndef FRANKACONTROL_ABSTRACTCONTROL_HPP
#define FRANKACONTROL_ABSTRACTCONTROL_HPP

#include <Eigen/Geometry>
#include <franka/model.h>
#include <franka/robot_state.h>
#include <memory>

namespace franka_control {
    namespace control {
        class AbstractControl {
        public:
            AbstractControl() = default;

            ~AbstractControl() = default;

            // set model
            void setModel(const std::shared_ptr<franka::Model>& model)
            {
                _model = model;
            }

            Eigen::Matrix<double, 6, 7> jacobian(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 6, 7>>(_model->zeroJacobian(franka::Frame::kEndEffector, state).data());
            }

            Eigen::Matrix<double, 7, 7> mass(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 7>>(_model->mass(state).data());
            }

            // Get joint position
            Eigen::Matrix<double, 7, 1> jointPosition(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());
            }

            // Get joint velocity
            Eigen::Matrix<double, 7, 1> jointVelocity(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data());
            }

            // Get joint torques
            Eigen::Matrix<double, 7, 1> jointTorques(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.tau_J.data());
            }

            // Get end-effector pose
            Eigen::Affine3d taskPose(const franka::RobotState& state)
            {
                return Eigen::Affine3d(Eigen::Matrix4d::Map(state.O_T_EE.data()));
            }

            // Get end-effector velocity
            Eigen::Matrix<double, 6, 1> taskVelocity(const franka::RobotState& state)
            {
                return jacobian(state) * jointVelocity(state);
            }

        protected:
            std::shared_ptr<franka::Model> _model;
        };

    } // namespace control

} // namespace franka_control

#endif // FRANKACONTROL_ABSTRACTCONTROL_HPP