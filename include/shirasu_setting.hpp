// The specification of "Shirasu" is reffered via the peges of "shirasu_fw" on GitHub.
#include <ros/ros.h>

namespace shirasu_setting
{
    // Each of the names below corresponds to the number, which makes a shirasu switch its mode status into the designated one when received.
    enum class BIDplus0_Cmd : uint8_t
    {
        disable_mode,  // 0
        default_mode,  // 1
        homing_mode,   // 2
        reserved_mode, // 3
        current_mode,  // 4
        velocity_mode, // 5
        position_mode, // 6
    };
}