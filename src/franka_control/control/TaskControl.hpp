#ifndef FRANKACONTROL_TASKCONTROL_HPP
#define FRANKACONTROL_TASKCONTROL_HPP

#include "franka_control/control/AbstractControl.hpp"

namespace franka_control {
    namespace control {
        class TaskControl : public AbstractControl {
        public:
            virtual Eigen::Matrix<double, 6, 1> action(const franka::RobotState& state) = 0;
        };

    } // namespace control

} // namespace franka_control

#endif // FRANKACONTROL_TASKCONTROL_HPP