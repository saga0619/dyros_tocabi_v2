#include "tocabi_controller/robot_data.h"

class SensorManager
{
public:
    SensorManager();
    ~SensorManager();

    void thread_imu();
    void thread_ft();
};
