#include <franka_control/Franka.hpp>

using namespace franka_control;

int main(int argc, char const* argv[])
{
    Franka robot("franka");

    std::string num_record = (argc > 1) ? argv[1] : "1";
    robot.record("outputs/record_" + num_record + ".csv", 100, 10);

    return 0;
}
