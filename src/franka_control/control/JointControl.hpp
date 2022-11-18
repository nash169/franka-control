#ifndef FRANKACONTROL_JOINTCONTROL_HPP
#define FRANKACONTROL_JOINTCONTROL_HPP

#include "franka_control/control/AbstractControl.hpp"

namespace franka_control {
    enum class ControlMode : unsigned int {
        OPERATIONSPACE = 1 << 0,
        CONFIGURATIONSPACE = 1 << 1
    };

    namespace control {
        class JointControl : public AbstractControl {
        public:
            JointControl(const ControlMode& mode = ControlMode::CONFIGURATIONSPACE) : _mode(mode) {}

            virtual Eigen::Matrix<double, 7, 1> action(const franka::RobotState& state) = 0;

        protected:
            ControlMode _mode;
        };

    } // namespace control

} // namespace franka_control

#endif // FRANKACONTROL_JOINTCONTROL_HPP