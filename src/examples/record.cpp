#include <franka_control/Franka.hpp>

using namespace franka_control;

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    robot.record("outputs/record_2.csv", 100, 10);

    return 0;
}
