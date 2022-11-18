#ifndef FRANKACONTROL_ABSTRACTCONTROL_HPP
#define FRANKACONTROL_ABSTRACTCONTROL_HPP

#include <Eigen/Core>
#include <franka/robot_state.h>

namespace franka_control {
    namespace control {
        class AbstractControl {
        public:
            AbstractControl() = default;

            ~AbstractControl() = default;

            // Get joint position
            Eigen::Matrix<double, 7, 1> jointPosition(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());
            }

            Eigen::Matrix<double, 7, 1> jointVelocity(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.dq.data());
            }

            Eigen::Matrix<double, 7, 1> jointTorques(const franka::RobotState& state)
            {
                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.tau_J.data());
            }

            Eigen::Matrix<double, 7, 1> taskPose(const franka::RobotState& state)
            {
                Eigen::Vector3d pos(state.O_T_EE[12], state.O_T_EE[13], state.O_T_EE[14]);
                Eigen::Matrix3d rot;
                rot << state.O_T_EE[0], state.O_T_EE[1], state.O_T_EE[2],
                    state.O_T_EE[4], state.O_T_EE[5], state.O_T_EE[6],
                    state.O_T_EE[8], state.O_T_EE[9], state.O_T_EE[10];

                return Eigen::Map<const Eigen::Matrix<double, 7, 1>>(state.q.data());
            }
        };

    } // namespace control

} // namespace franka_control

#endif // FRANKACONTROL_ABSTRACTCONTROL_HPP