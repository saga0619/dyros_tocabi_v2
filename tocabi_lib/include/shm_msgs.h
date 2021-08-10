#ifndef SHM_MSGS_H
#define SHM_MSGS_H

#include <pthread.h>
#include <atomic>
#include <sys/shm.h>
#include <sys/ipc.h>
#include <time.h>

//per link
//Jac * 4
//33 * 6 * 39 * 4

#define MODEL_DOF 33

typedef struct SHMmsgs
{
    int8_t safety_status[MODEL_DOF];
    int8_t ecat_status[MODEL_DOF];
    int8_t zp_status[MODEL_DOF];

    long tv_sec;
    long tv_nsec;
    std::atomic<bool> lowerReady;
    std::atomic<bool> ecatTimerSet;

    std::atomic<int> statusCount;
    std::atomic<int> statusCount2;
    std::atomic<int> statusWriting;
    std::atomic<bool> triggerS1;

    struct timespec ts;

    int status[MODEL_DOF];
    float torqueActual[MODEL_DOF];
    float vel[MODEL_DOF];
    float pos[MODEL_DOF];
    float posExt[MODEL_DOF];
    std::atomic_int16_t elmo_torque[MODEL_DOF];

    float sim_time_;

    float pos_virtual[7]; //virtual pos(3) + virtual quat(4)
    float vel_virtual[6]; //virtual vel(3) + virtual twist(3)
    float imu_acc[3];

    std::atomic<bool> imuWriting;
    float imuRaw[6];

    std::atomic<bool> ftWriting;
    float ftSensor[12];

    int imu_state;
    int ft_state;

    //command val

    std::atomic<bool> commanding;
    std::atomic<int> commandCount;
    std::atomic<int> stloopCount;
    int commandMode[MODEL_DOF]; //command mode 0 -> off 1 -> torque 2 -> position
    float torqueCommand[MODEL_DOF];
    float positionCommand[MODEL_DOF];

    int maxTorque = 0;

    float timeCommand;

    std::atomic<int> control_time_us_;
    
    std::atomic<int> t_cnt;
    std::atomic<int> t_cnt2;
    std::atomic<bool> controllerReady;
    std::atomic<bool> reading;
    std::atomic<int> process_num;
    std::atomic<bool> shutdown; //true for exit
    std::atomic<bool> emergencyOff;
    std::atomic<bool> controlModeLower;
    std::atomic<bool> controlModeUpper;
    std::atomic<bool> safety_disable;

    long std_timer_ns;

    std::atomic<bool> upperTimerSet;
    std::atomic<bool> lowerTimerSet;
    

    float lat_avg, lat_min, lat_max, lat_dev;
    float send_avg, send_min, send_max, send_dev;

    float lat_avg2, lat_min2, lat_max2, lat_dev2;
    float send_avg2, send_min2, send_max2, send_dev2;

    bool low_init_signal = false;
    bool waist_init_signal = false;
    bool upper_init_signal = false;

    std::atomic<bool> safety_reset_lower_signal;
    std::atomic<bool> safety_reset_upper_signal;
    bool force_load_saved_signal = false;

} SHMmsgs;

//static SHMmsgs *shm_msgs_;

static const key_t shm_msg_key = 10561;
static const key_t shm_rd_key = 10334;

// const std::string cred("\033[0;31m");
// const std::string creset("\033[0m");
// const std::string cblue("\033[0;34m");
// const std::string cgreen("\033[0;32m");
// const std::string cyellow("\033[0;33m");

enum ECOMMAND
{
    POSITION = 11,
    TORQUE = 22
};

// Joint state
// joint state is indicated with 3 numbers

// 0 : ELMO_ERROR,
// 1 : OPERATION_READY,
// 2 : COMMUTATION_INITIALIZE,
// 3 : COMMUTATION_DONE, 4

// 0 : ZP_SEARCHING_ZP,
// 1 : ZP_SEARCH_COMPLETE,
// 2 : ZP_MANUAL_REQUIRED,
// 3 : ZP_NOT_ENOUGH_HOMMING,
// 4 : ZP_GOTO_ZERO,
// 5 : ZP_SUCCESS, 8

// 0 : SAFETY_VELOCITY_LIMIT,
// 1 : SAFETY_JOINT_LIMIT,
// 2 : SAFETY_TORQUE_LIMIT, 3

enum SSTATE
{
    SAFETY_OK,
    SAFETY_JOINT_LIMIT,
    SAFETY_VELOCITY_LIMIT,
    SAFETY_TORQUE_LIMIT,
    SAFETY_COMMAND_LOCK,
};

enum ZSTATE
{
    ZP_SEARCHING_ZP,
    ZP_SEARCH_COMPLETE,
    ZP_MANUAL_REQUIRED,
    ZP_NOT_ENOUGH_HOMMING,
    ZP_GOTO_ZERO,
    ZP_SUCCESS,
};

enum ESTATE
{
    ERROR,
    OPERATION_READY,
    COMMUTATION_INITIALIZE,
    COMMUTATION_DONE,
};

static void init_shm(int shm_key, int &shm_id_, SHMmsgs **shm_ref)
{
    if ((shm_id_ = shmget(shm_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
        exit(0);
    }
    if ((*shm_ref = (SHMmsgs *)shmat(shm_id_, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed " << std::endl;
        exit(0);
    }

    if ((*shm_ref)->process_num == 0)
    {
        std::cout << "\033[0;32m" << "Process num 0 ! Clean Start!" << "\033[0m" << std::endl;
    }

    (*shm_ref)->process_num++;
}

static void deleteSharedMemory(int shm_id__, SHMmsgs *shm_ref)
{
    shm_ref->process_num--;
    if (shm_ref->process_num == 0)
    {
        printf("process num 0. removing shared memory\n");

        if (shmctl(shm_id__, IPC_RMID, NULL) == -1)
        {
            printf("shared memoty failed to remove. \n");
        }
        else
        {
            printf("Shared memory succesfully removed\n");
        }
    }
}

// static void init_shm_master()
// {

//     if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
//     {
//         std::cout << "shm mtx failed " << std::endl;
//         exit(0);
//     }

//     if ((shm_msgs_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
//     {
//         std::cout << "shmat failed " << std::endl;
//         exit(0);
//     }

//     if (shmctl(shm_msg_id, SHM_LOCK, NULL) == 0)
//     {
//         //std::cout << "SHM_LOCK enabled" << std::endl;
//     }
//     else
//     {
//         std::cout << "SHM lock failed" << std::endl;
//     }

//     shm_msgs_->t_cnt = 0;
//     shm_msgs_->t_cnt2 = 0;
//     shm_msgs_->controllerReady = false;
//     shm_msgs_->statusWriting = 0;
//     shm_msgs_->commanding = false;
//     shm_msgs_->reading = false;
//     shm_msgs_->shutdown = false;

//     //
//     //float lat_avg, lat_min, lat_max, lat_dev;
//     //float send_avg, send_min, send_max, send_dev;

//     shm_msgs_->lat_avg2 = 0;
//     shm_msgs_->lat_min2 = 0;
//     shm_msgs_->lat_max2 = 100000;
//     shm_msgs_->lat_dev2 = 0;

//     shm_msgs_->send_avg2 = 0;
//     shm_msgs_->send_min2 = 0;
//     shm_msgs_->send_max2 = 100000;
//     shm_msgs_->send_dev2 = 0;

//     //std::cout << "shm master initialized" << std::endl;
// }

// void SendCommand(float *torque_command, float *position_command, int *mode)
// {
//     shm_msgs_->commanding = true;

//     shm_msgs_->commanding = false;
// }

#endif