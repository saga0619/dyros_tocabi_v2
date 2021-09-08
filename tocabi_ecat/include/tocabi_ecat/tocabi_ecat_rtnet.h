
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

int start_joint_ = 0;

int stateElmo[ELMO_DOF];
int stateElmo_before[ELMO_DOF];

bool torqueCCEnable;
double torqueCC_recvt;
double torqueCC_comt;

double control_time_real_;

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

int wait_cnt = 0;

int commutation_joint = 0;
long cycle_count = 0;

volatile bool de_operation_ready;
volatile bool de_emergency_off;
volatile bool de_shutdown;
volatile bool de_ecat_lost;
volatile bool de_ecat_lost_before;
volatile bool de_ecat_recovered;
volatile bool de_initialize;
volatile bool de_commutation_done;
volatile bool de_zp_sequence;
volatile bool de_zp_upper_switch;
volatile bool de_zp_lower_switch;
volatile int de_debug_level;

int8_t state_elmo_[ELMO_DOF];
int8_t state_zp_[ELMO_DOF];
int8_t state_safety_[ELMO_DOF];

bool force_control_mode = false;
int soem_freq = 0;
int expected_counter = 0;
int period_ns = 0;

int joint_state_elmo_[ELMO_DOF]; //sendstate
int joint_state_[ELMO_DOF];      //sendstate

float q_elmo_[ELMO_DOF];      //sendstate
float q_dot_elmo_[ELMO_DOF];  //sendstate
float torque_elmo_[ELMO_DOF]; //sendstate
float q_ext_elmo_[ELMO_DOF];

float q_[ELMO_DOF];      //sendstate
float q_dot_[ELMO_DOF];  //sendstate
float torque_[ELMO_DOF]; //sendstate
float q_ext_[ELMO_DOF];

int command_mode_[ELMO_DOF];
float torque_desired_elmo_[ELMO_DOF]; //get torque command
float q_desired_elmo_[ELMO_DOF];      //get joint command
float torque_desired_[ELMO_DOF];      //get torque command
float q_desired_[ELMO_DOF];           //get joint command

double q_zero_point[ELMO_DOF];

double q_zero_elmo_[ELMO_DOF];
double q_zero_mod_elmo_[ELMO_DOF];

int maxTorque = 0;

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

int shm_id_;
SHMmsgs *shm_msgs_;

bool saveCommutationLog();
bool loadCommutationLog(struct timespec &commutation_time);

bool saveZeroPoint();
bool loadZeroPoint(bool force = false);

bool status_log = false;

void emergencyOff();

int kbhit(void);

int getElmoState(uint16_t state_bit);

void findzeroLeg();
void findZeroPointlow(int slv_number, double time_real_);
void findZeroPoint(int slv_number, double time_real_);

const char cred[] = "\033[0;31m";
const char creset[] = "\033[0m";
const char cblue[] = "\033[0;34m";
const char cgreen[] = "\033[0;32m";
const char cyellow[] = "\033[0;33m";

int lat_avg, lat_min, lat_max, lat_dev;
int send_avg, send_min, send_max, send_dev;

float lat_avg2, lat_min2, lat_max2, lat_dev2;
float send_avg2, send_min2, send_max2, send_dev2;

int low_rcv_ovf, low_mid_ovf, low_snd_ovf;
int low_rcv_us, low_mid_us, low_snd_us;
float low_rcv_avg, low_rcv_max;
float low_mid_avg, low_mid_max;
float low_snd_avg, low_snd_max;

unsigned long long g_cur_dc32 = 0;
unsigned long long g_pre_dc32 = 0;
long long g_diff_dc32 = 0;
long long g_cur_DCtime = 0, g_max_DCtime = 0;
int g_PRNS = period_ns;
struct timespec g_ts;
int64 g_toff; //, gl_delta;

void getErrorName(int err_register, char *err);
TocabiInitArgs g_init_args;

long getTimeDiff(struct timespec &from, struct timespec &to);
long getTimeDiff(struct timespec &a);