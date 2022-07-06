#ifndef FRANKACONTROL_FRANKA_HPP
#define FRANKACONTROL_FRANKA_HPP

#include <franka/model.h>
#include <franka/robot.h>

namespace franka_control {
    class Franka {
    public:
        Franka(/* args */);
        ~Franka();

    protected:
        franka::Robot* _robot;
        franka::Model* _model;
    };
} // namespace franka_control

#endif // FRANKACONTROL_FRANKA_HPP