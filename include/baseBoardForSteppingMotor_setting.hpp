
#include <ros/ros.h>

namespace baseBoardForSteppingMotor_setting
{

    enum class BIDplus0_Cmd : uint8_t
    {
        disable_mode=0,
        default_mode = 1,
        // homing_mode,
        // reserved_mode,
        // current_mode,
        position_mode = 4,
        velocity_mode = 5,
    };
}