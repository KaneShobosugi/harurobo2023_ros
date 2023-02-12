
#include <ros/ros.h>

namespace baseBoardForSteppingMotor_setting
{

    enum class BIDplus0_Cmd : uint8_t
    {
        // disable_mode,
        default_mode = 0,
        // homing_mode,
        // reserved_mode,
        // current_mode,
        // velocity_mode,
        position_mode = 4,
    };
}