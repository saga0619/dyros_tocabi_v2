#ifndef TOCABI_CONTROLLER_H
#define TOCABI_CONTROLLER_H

#include "tocabi_controller/state_manager.h"
#include "wholebody_functions.h"

#include "tocabi_msgs/TaskGainCommand.h"

#ifdef COMPILE_TOCABI_CC
#include "cc.h"
#endif
#ifdef COMPILE_TOCABI_AVATAR
#include "avatar.h"
#endif

class TocabiController
{
public:
    TocabiController(StateManager &stm_global);

    ~TocabiController();
    void *Thread1();
    void *Thread2();
    void *Thread3();

    DataContainer &dc_;
    StateManager &stm_;
    RobotData &rd_;

#ifdef COMPILE_TOCABI_CC
    CustomController &my_cc;
#endif

#ifdef COMPILE_TOCABI_AVATAR
    AvatarController &ac_;
#endif

    static void *Thread1Starter(void *context)
    {
        return ((TocabiController *)context)->Thread1();
    }
    static void *Thread2Starter(void *context) { return ((TocabiController *)context)->Thread2(); }
    static void *Thread3Starter(void *context) { return ((TocabiController *)context)->Thread3(); }

    void SendCommand(Eigen::VectorQd torque_command);
    void GetTaskCommand(tocabi_msgs::TaskCommand &msg);

    void MeasureTime(int currentCount, int nanoseconds1, int nanoseconds2 = 0);
    int64_t total1 = 0, total2 = 0, total_dev1 = 0, total_dev2 = 0;
    float lmax = 0.0, lmin = 10000.00, ldev = 0.0, lavg = 0.0, lat = 0.0;
    float smax = 0.0, smin = 10000.00, sdev = 0.0, savg = 0.0, sat = 0.0;

    std::atomic<bool> enableThread2;
    void EnableThread2(bool enable);
    std::atomic<bool> enableThread3;
    void EnableThread3(bool enable);

    void RequestThread2();
    void RequestThread3();

    std::atomic<bool> signalThread1;
    std::atomic<bool> triggerThread2;
    std::atomic<bool> triggerThread3;

    ros::NodeHandle nh_controller_;
    ros::CallbackQueue queue_controller_;

    ros::Subscriber task_command_que_sub_;
    tocabi_msgs::TaskCommandQue tc_que_msg_;
    ros::Subscriber task_command_sub_;
    tocabi_msgs::TaskCommand tc_msg_;
    ros::Subscriber position_command_sub_;

    tocabi_msgs::TaskGainCommand tcg_msg_;
    ros::Subscriber task_gain_sub_;

    void PositionCommandCallback(const tocabi_msgs::positionCommandConstPtr &msg);
    void TaskCommandCallback(const tocabi_msgs::TaskCommandConstPtr &msg);
    void TaskQueCommandCallback(const tocabi_msgs::TaskCommandQueConstPtr &msg);
    void TaskGainCommandCallback(const tocabi_msgs::TaskGainCommandConstPtr &msg);


    //CustomController Linker
    void QueCustomController();
    void WaitCustomControllerCommand();
};

#endif