#ifndef FRANKACONTROL_TOOLS_COMMON_HPP
#define FRANKACONTROL_TOOLS_COMMON_HPP

#include <franka/robot.h>

namespace franka_control {
    namespace tools {
        void setDefaultBehavior(franka::Robot& robot)
        {
            robot.setCollisionBehavior(
                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}},
                {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}}, {{1000.0, 1000.0, 1000.0, 1000.0, 1000.0, 1000.0}});

            // robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});

            // robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
        }
    } // namespace tools

} // namespace franka_control

#endif // FRANKACONTROL_TOOLS_COMMON_HPP