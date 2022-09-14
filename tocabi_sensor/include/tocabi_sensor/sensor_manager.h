#include <std_msgs/String.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include "tocabi_sensor/ati_ft_ethernet.h"
#include "tocabi_sensor/sensoray826.h"
#include "tocabi_sensor/mx5_imu.h"
#include <fstream>
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
    ros::Publisher imu_pub;
    ros::Publisher fthand_pub;
    
    void GuiCommandCallback(const std_msgs::StringConstPtr &msg);

    bool imu_reset_signal_ = false;
    bool ft_calib_signal_ = false;
    bool handft_calib_signal_ = false;

    std::fstream ft_init_log;

    int packet_num;
};
