#include <std_msgs/String.h>
#include <std_msgs/Int32MultiArray.h>

#include "tocabi_data/robot_data.h"
#include "tocabi_controller/mx5_imu.h"

class SensorManager
{
public:
    SensorManager();
    ~SensorManager() {}

    void *IMUThread(void);
    static void *IMUthread_starter(void *context) { return ((SensorManager *)context)->IMUThread(); }

    void *FTThread(void);
    static void *FTthread_starter(void *context) { return ((SensorManager *)context)->FTThread(); }

    SHMmsgs *shm_;
    ros::NodeHandle nh_;

    ros::Subscriber gui_command_sub_;
    ros::Publisher gui_state_pub_;
    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);

    bool imu_reset_ = false;
};
