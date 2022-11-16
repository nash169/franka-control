#include <franka_control/Franka.hpp>

using namespace franka_control;

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    robot.record("robot_state.csv");

    return 0;
}
