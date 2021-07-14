#include <iostream>
#include <thread>
#include <atomic>
#include <chrono>
#include <vector>

#include <stdarg.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <fstream>
#include <sys/ipc.h>
#include <sys/shm.h>
#include <cstring>
#include <cmath>
#include <sys/stat.h>

#include <ethercat.h>
#include <ros/ros.h>
#include "tocabi_ecat/ecat_settings.h"
#include "shm_msgs.h"

using namespace std;

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

const int FAULT_BIT = 3;
const int OPERATION_ENABLE_BIT = 2;
const int SWITCHED_ON_BIT = 1;
const int READY_TO_SWITCH_ON_BIT = 0;

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

ElmoHomming elmofz[ELMO_DOF];
ElmoState elmost[ELMO_DOF];
int ElmoMode[ELMO_DOF];
enum
{
    EM_POSITION = 11,
    EM_TORQUE = 22,
    EM_DEFAULT = 33,
    EM_COMMUTATION = 44,
};

char IOmap[4096];
OSAL_THREAD_HANDLE thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

char commutation_cache_file[] = "/home/dyros/.tocabi_bootlog/commutationlog_upper";
char zeropoint_cache_file[] = "/home/dyros/.tocabi_bootlog/zeropointlog_upper";
int stateElmo[ELMO_DOF];
int stateElmo_before[ELMO_DOF];

bool torqueCCEnable;
double torqueCC_recvt;
double torqueCC_comt;

double control_time_real_;
int control_time_us_;
double torque_on_time_;
double torque_off_time_;
bool torque_switch_;
bool torque_on_;
bool torque_off_;

bool hommingElmo[ELMO_DOF];
bool hommingElmo_before[ELMO_DOF];

enum SAFETY_PROTOCOL
{
    NORMAL,
    LOCKED_BY_VEL,
    LOCKED_BY_JOL,
    DISABLED,
};
int ElmoSafteyMode[ELMO_DOF];

EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_rx *rxPDO[ELMO_DOF];
EtherCAT_Elmo::ElmoGoldDevice::elmo_gold_tx *txPDO[ELMO_DOF];

bool ElmoConnected = false;
bool ElmoTerminate = false;

int fz_group1[18] = {
    Neck_Joint, Head_Joint,
    R_Shoulder1_Joint, R_Shoulder2_Joint, R_Shoulder3_Joint, R_Armlink_Joint, R_Elbow_Joint, R_Forearm_Joint, R_Wrist1_Joint, R_Wrist2_Joint,
    L_Shoulder1_Joint, L_Shoulder2_Joint, L_Shoulder3_Joint, L_Armlink_Joint, L_Elbow_Joint, L_Forearm_Joint, L_Wrist1_Joint, L_Wrist2_Joint};

int fz_group2[3] = {
    Upperbody_Joint, Waist1_Joint, Waist2_Joint};

int fz_group3[12] = {
    R_HipYaw_Joint, R_HipRoll_Joint, R_HipPitch_Joint, R_Knee_Joint, R_AnklePitch_Joint, R_AnkleRoll_Joint,
    L_HipYaw_Joint, L_HipRoll_Joint, L_HipPitch_Joint, L_Knee_Joint, L_AnklePitch_Joint, L_AnkleRoll_Joint};

bool fz_group1_check = false;
bool fz_group2_check = false;
bool fz_group3_check = false;
int fz_group = 0;

bool ConnectionUnstableBeforeStart = false;

int bootseq = 0;
//int bootseq
const int firstbootseq[5] = {0, 33, 35, 8, 64};
const int secondbootseq[4] = {0, 33, 35, 39};

bool ecat_connection_ok = false;

bool ecat_number_ok = false;
bool ecat_WKC_ok = false;
bool commutation_check = true;
bool commutation_ok = false;
bool commutation_fail = false;

bool zp_waiting_low_switch = false;
bool zp_waiting_upper_switch = false;

bool zp_init_check = true;
bool zp_low_check = false;
bool zp_upper_check = false;
bool zp_ok = false;
bool zp_fail = false;
bool zp_load_ok = true;

bool wait_kill_switch = true;
bool wait_time_over = false;
bool check_commutation = true;
bool check_commutation_first = true;
bool query_check_state = false;
bool zp_lower_calc = true;

bool ecat_verbose = true;

int wait_cnt = 0;

int commutation_joint = 0;

chrono::steady_clock::time_point st_start_time;
std::chrono::duration<double> time_from_begin;
std::chrono::nanoseconds cycletime(PERIOD_NS);
int cycle_count = 0;

atomic<bool> de_operation_ready{false};
atomic<bool> de_emergency_off{false};
atomic<bool> de_shutdown{false};
atomic<bool> de_ecat_lost{false};
atomic<bool> de_ecat_lost_before{false};
atomic<bool> de_ecat_recovered{false};
atomic<bool> de_initialize{false};
atomic<bool> de_commutation_done{false};
atomic<bool> de_zp_sequence{false};
atomic<bool> de_zp_upper_switch{false};
atomic<bool> de_zp_lower_switch{false};
atomic<int> de_debug_level{0};


int8_t state_elmo_[ELMO_DOF];
int8_t state_zp_[ELMO_DOF];
int8_t state_safety_[ELMO_DOF];

int joint_state_elmo_[ELMO_DOF]; //sendstate


float q_elmo_[ELMO_DOF];      //sendstate
float q_dot_elmo_[ELMO_DOF];  //sendstate
float torque_elmo_[ELMO_DOF]; //sendstate
float q_ext_elmo_[ELMO_DOF];

int command_mode_elmo_[ELMO_DOF];       //0 off 1 torque 2 position 
float torque_desired_elmo_[ELMO_DOF];   //get torque command
float q_desired_elmo_[ELMO_DOF];        //get joint command

double q_zero_point[ELMO_DOF];

double q_zero_elmo_[ELMO_DOF];
double q_zero_mod_elmo_[ELMO_DOF];

//from param
// std::vector<double> NM2CNT;
// std::vector<double> joint_velocity_limit;
// std::vector<double> joint_upper_limit;
// std::vector<double> joint_lower_limit;

int maxTorque = 0;

int joint_state_[ELMO_DOF]; //sendstate
float q_[ELMO_DOF];      //sendstate
float q_dot_[ELMO_DOF];  //sendstate
float torque_[ELMO_DOF]; //sendstate
float q_ext_[ELMO_DOF];

int command_mode_[ELMO_DOF];
float torque_desired_[ELMO_DOF]; //get torque command
float q_desired_[ELMO_DOF];      //get joint command


void *ethercatThread1(void *data);
void *ethercatThread2(void *data);
void ethercatCheck();

double elmoJointMove(double init, double angle, double start_time, double traj_time);
bool controlWordGenerate(const uint16_t statusWord, uint16_t &controlWord);

void elmoInit();

void checkJointSafety();
void checkJointStatus();

void initSharedMemory();
void sendJointStatus();
void getJointCommand();
void deleteSharedMemory();

bool saveCommutationLog();
bool loadCommutationLog();

//bool commutation_time_loaded = false;

chrono::system_clock::time_point commutation_save_time_;

bool saveZeroPoint();
bool loadZeroPoint(bool force = false);

void emergencyOff();

int kbhit(void);

int getElmoState(uint16_t state_bit);

void findzeroLeg();
void findZeroPointlow(int slv_number);
void findZeroPoint(int slv_number);

const std::string cred("\033[0;31m");
const std::string creset("\033[0m");
const std::string cblue("\033[0;34m");
const std::string cgreen("\033[0;32m");
const std::string cyellow("\033[0;33m");
