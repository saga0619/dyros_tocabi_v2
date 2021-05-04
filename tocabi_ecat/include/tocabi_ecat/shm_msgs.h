#ifndef SHM_MSGS_H
#define SHM_MSGS_H

#include <pthread.h>
#include <atomic>
#include <sys/shm.h>
#include <sys/ipc.h>

//per link
//Jac * 4
//33 * 6 * 39 * 4

#define MODEL_DOF 33

typedef struct SHMmsgs
{
    int status[MODEL_DOF];

    std::atomic<int> statusWriting;
    float torqueActual[MODEL_DOF];
    float vel[MODEL_DOF];
    float pos[MODEL_DOF];
    float posExt[MODEL_DOF];

    float sim_time_;

    float pos_virtual[7]; //virtual pos(3) + virtual quat(4)
    float vel_virtual[6]; //virtual vel(3) + virtual twist(3)

    float imuRaw[6];
    float ftSensor[12];

    //command val

    std::atomic<bool> commanding;
    int commandMode[MODEL_DOF];
    float torqueCommand[MODEL_DOF];
    float positionCommand[MODEL_DOF];

    float timeCommand;

    std::atomic<int> t_cnt;
    std::atomic<int> t_cnt2;
    std::atomic<bool> controllerReady;
    std::atomic<bool> reading;

    std::atomic<bool> shutdown;

    float lat_avg, lat_min, lat_max, lat_dev;
    float send_avg, send_min, send_max, send_dev;

    float lat_avg2, lat_min2, lat_max2, lat_dev2;
    float send_avg2, send_min2, send_max2, send_dev2;

} SHMmsgs;

static SHMmsgs *shm_msgs_;
static int shm_msg_id;
static key_t shm_msg_key = 7056;

enum ECOMMAND
{
    POSITION = 11,
    TORQUE = 22
};

// Joint state
// 0 : ELMO_ERROR,
// 1 : OPERATION_READY,
// 2 : COMMUTATION_INITIALIZE,
// 3 : COMMUTATION_DONE,
// 4 : ZP_SEARCHING_ZP,
// 5 : ZP_SEARCH_COMPLETE,
// 6 : ZP_MANUAL_REQUIRED,
// 7 : ZP_NOT_ENOUGH_HOMMING,
// 8 : ZP_GOTO_ZERO,
// 9 : ZP_SUCCESS,
// 10 : SAFETY_VELOCITY_LIMIT,
// 11 : SAFETY_JOINT_LIMIT,
// 12 : SAFETY_TORQUE_LIMIT,
enum ESTATE
{
    ERROR,
    OPERATION_READY,
    COMMUTATION_INITIALIZE,
    COMMUTATION_DONE,
    ZP_SEARCHING_ZP,
    ZP_SEARCH_COMPLETE,
    ZP_MANUAL_REQUIRED,
    ZP_NOT_ENOUGH_HOMMING,
    ZP_GOTO_ZERO,
    ZP_SUCCESS,
    SAFETY_VELOCITY_LIMIT,
    SAFETY_JOINT_LIMIT,
    SAFETY_TORQUE_LIMIT,
};

static void init_shm()
{

    if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
        exit(0);
    }

    if ((shm_msgs_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed " << std::endl;
        exit(0);
    }
    shm_msgs_->shutdown = false;
}

static void deleteSharedMemory()
{
    if (shmctl(shm_msg_id, IPC_RMID, NULL) == -1)
    {
        printf("shmctl failed\n");
    }
    else
    {
        printf("shm cleared succesfully\n");
    }
}

static void init_shm_master()
{

    if ((shm_msg_id = shmget(shm_msg_key, sizeof(SHMmsgs), IPC_CREAT | 0666)) == -1)
    {
        std::cout << "shm mtx failed " << std::endl;
        exit(0);
    }

    if ((shm_msgs_ = (SHMmsgs *)shmat(shm_msg_id, NULL, 0)) == (SHMmsgs *)-1)
    {
        std::cout << "shmat failed " << std::endl;
        exit(0);
    }

    if (shmctl(shm_msg_id, SHM_LOCK, NULL) == 0)
    {
        //std::cout << "SHM_LOCK enabled" << std::endl;
    }
    else
    {
        std::cout << "SHM lock failed" << std::endl;
    }

    shm_msgs_->t_cnt = 0;
    shm_msgs_->t_cnt2 = 0;
    shm_msgs_->controllerReady = false;
    shm_msgs_->statusWriting = 0;
    shm_msgs_->commanding = false;
    shm_msgs_->reading = false;
    shm_msgs_->shutdown = false;

    //
    //float lat_avg, lat_min, lat_max, lat_dev;
    //float send_avg, send_min, send_max, send_dev;

    shm_msgs_->lat_avg2 = 0;
    shm_msgs_->lat_min2 = 0;
    shm_msgs_->lat_max2 = 100000;
    shm_msgs_->lat_dev2 = 0;

    shm_msgs_->send_avg2 = 0;
    shm_msgs_->send_min2 = 0;
    shm_msgs_->send_max2 = 100000;
    shm_msgs_->send_dev2 = 0;

    //std::cout << "shm master initialized" << std::endl;
}

// void SendCommand(float *torque_command, float *position_command, int *mode)
// {
//     shm_msgs_->commanding = true;

//     shm_msgs_->commanding = false;
// }

#endif