#pragma once

// #include <atomic>
#include <stdio.h>
#include <string.h>
#include <stdarg.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ipc.h>
#include <sys/shm.h>
// #include <cstring>
#include <sys/stat.h>
#include <math.h>
#include <ethercat.h>
#include "tocabi_ecat/ecat_settings.h"
#include "shm_msgs.h"

struct TocabiInitArgs
{
    char port1[20];
    char port2[20];

    int period_ns;
    int lock_core;

    int ecat_device; // ELMO 1, ELMO 2
    int ecat_slave_num;
    int ecat_slave_start_num;

    bool is_main;

    bool verbose;

    char commutation_cache_file[100]; // = "/home/dyros/.tocabi_bootlog/commutationlog";
    char zeropoint_cache_file[100];   // = "/home/dyros/.tocabi_bootlog/zeropointlog";
};

namespace EtherCAT_Elmo
{
    enum MODE_OF_OPERATION
    {
        ProfilePositionmode = 1,
        ProfileVelocitymode = 3,
        ProfileTorquemode = 4,
        Homingmode = 6,
        InterpolatedPositionmode = 7,
        CyclicSynchronousPositionmode = 8,
        CyclicSynchronousVelocitymode = 9,
        CyclicSynchronousTorquemode = 10,
        CyclicSynchronousTorquewithCommutationAngle = 11
    };

    struct ElmoGoldDevice
    {
        struct elmo_gold_tx
        {
            int32_t targetPosition;
            int32_t targetVelocity;
            int16_t targetTorque;
            uint16_t maxTorque;
            uint16_t controlWord;
            int8_t modeOfOperation;
        };
        struct elmo_gold_rx
        {
            int32_t positionActualValue;
            //int32_t positionFollowingErrrorValue;
            uint32_t hommingSensor;
            uint16_t statusWord;
            //int8_t modeOfOperationDisplay;
            int32_t velocityActualValue;
            int16_t torqueActualValue;
            //int16_t torqueDemandValue;
            int32_t positionExternal;
        };
    };
} // namespace EtherCAT_Elmo

enum
{
    CW_SHUTDOWN = 6,
    CW_SWITCHON = 7,
    CW_ENABLEOP = 15,
    CW_DISABLEOP = 7,
};

enum
{
    ELMO_FAULT = 0,
    ELMO_NOTFAULT = 2,
    ELMO_READY_TO_SWITCH_ON = 3,
    ELMO_SWITCHED_ON = 4,
    ELMO_OPERATION_ENABLE = 1,
};

namespace ElmoHommingStatus
{
    enum FZResult
    {
        SUCCESS = 1,
        FAILURE = 0
    };
}; // namespace ElmoHommingStatus

struct ElmoState
{
    int boot_sequence = 0;
    int state = 0;
    int state_before = 0;
    uint16_t check_value = 0;
    uint16_t check_value_before = 0;

    bool commutation_ok = false;
    bool commutation_required = false;
    bool commutation_not_required = false;
    bool first_check = true;
};

struct ElmoHomming
{
    bool hommingElmo;
    bool hommingElmo_before;
    bool startFound = false;
    bool endFound = false;
    int findZeroSequence = 0;
    double initTime;
    double initPos;
    double desPos;
    double posStart;
    double posEnd;
    double req_length = 0.2;
    double firstPos;
    double init_direction = 1;
    int status;
    int result;
};

enum ElmoJointState
{
    MOTOR_COMMUTATION = 1,
    MOTOR_OPERATION_READY = 2,
    MOTOR_SEARChING_REQUIRED = 3,
    MOTOR_SEARCHING_ZP = 4,
    MOTOR_SEARCHING_MANUAL = 5,
    MOTOR_SEARCHING_COMPLETE = 6,
    MOTOR_SAFETY_LOCK = 7,
    MOTOR_SAFETY_DISABLED = 8,
};

enum
{
    FZ_CHECKHOMMINGSTATUS,
    FZ_FINDHOMMINGSTART,
    FZ_FINDHOMMINGEND,
    FZ_FINDHOMMING,
    FZ_GOTOZEROPOINT,
    FZ_HOLDZEROPOINT,
    FZ_FAILEDANDRETURN,
    FZ_MANUALDETECTION,
    FZ_TORQUEZERO,
};

enum
{
    EM_POSITION = 11,
    EM_TORQUE = 22,
    EM_DEFAULT = 33,
    EM_COMMUTATION = 44,
};

enum SAFETY_PROTOCOL
{
    NORMAL,
    LOCKED_BY_VEL,
    LOCKED_BY_JOL,
    DISABLED,
};

void *ethercatThread1(void *data);
void *ethercatThread2(void *data);
void ethercatCheck();

double elmoJointMove(double init, double angle, double start_time, double traj_time);

bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);
void checkFault(const uint16_t statusWord, int slave);

bool initTocabiSystem(const TocabiInitArgs &args);
void cleanupTocabiSystem();

void elmoInit();

void checkJointSafety();
void checkJointStatus();

//void initSharedMemory();
void sendJointStatus();
void getJointCommand();

bool saveCommutationLog();
bool loadCommutationLog(struct timespec &commutation_time);

bool saveZeroPoint();
bool loadZeroPoint(bool force = false);


void emergencyOff();

int kbhit(void);

int getElmoState(uint16_t state_bit);

void findzeroLeg();
void findZeroPointlow(int slv_number, double time_real_);
void findZeroPoint(int slv_number, double time_real_);

void getErrorName(int err_register, char *err);

long getTimeDiff(struct timespec &from, struct timespec &to);
long getTimeDiff(struct timespec &a);