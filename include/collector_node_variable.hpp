#include <ros/ros.h>
#include <math.h>

namespace collector_node_variable
{
    const uint16_t baseBoardForSteppingMotorID{0x300};
    const float collection_steppingMotorPosDisplacement{7 * M_PI};
    const float collection_armRotationTime{3};
    const float ejectionTime{5};
    const float ejectionleverRotationTime{10};

    const float collection_steppingVel{5* M_PI};
    

}