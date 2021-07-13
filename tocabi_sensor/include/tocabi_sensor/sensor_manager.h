#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include "sensoray826/sensoray826.h"
#include "tocabi_sensor/mx5_imu.h"
#include "shm_msgs.h"

class SensorManager
{
public:
    SensorManager();
    ~SensorManager() {}

    void *SensorThread(void);
    static void *SensorThread_starter(void *context) { return ((SensorManager *)context)->SensorThread(); }

    // void *FTThread(void);
    // static void *FTthread_starter(void *context) { return ((SensorManager *)context)->FTThread(); }

    SHMmsgs *shm_;
    ros::NodeHandle nh_;

    ros::Subscriber gui_command_sub_;
    ros::Publisher gui_state_pub_;
    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);

    bool imu_reset_signal_ = false;
    bool ft_calib_signal_ = false;
};
