#ifndef TOCABI_CONTROLLER_H
#define TOCABI_CONTROLLER_H

#include "tocabi_controller/state_manager.h"
#include "tocabi_controller/wholebody_functions.h"

class TocabiController
{
public:
    TocabiController(DataContainer &dc_global, StateManager &stm_global);

    ~TocabiController();
    void *thread1();
    void *thread2();
    void *thread3();

    DataContainer &dc_;
    StateManager &stm_;
    RobotData &rd_;

    static void *thread1_starter(void *context) { return ((TocabiController *)context)->thread1(); }
    static void *thread2_starter(void *context) { return ((TocabiController *)context)->thread2(); }
    static void *thread3_starter(void *context) { return ((TocabiController *)context)->thread3(); }

    void SendCommand(Eigen::VectorQd torque_command);
    void GetTaskCommand(tocabi_msgs::TaskCommand &msg);
};

#endif