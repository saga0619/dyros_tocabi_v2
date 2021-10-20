
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
#include "ecat_settings.h"
#include "shm_msgs.h"

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
