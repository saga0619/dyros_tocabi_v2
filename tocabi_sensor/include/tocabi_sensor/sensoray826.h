#include "sensoray826/lib/826api.h"
#include "math_type_define.h"
#include <ros/ros.h>
#include <thread>

#define PRINT_ERR(FUNC)                       \
    if ((errcode = FUNC) != S826_ERR_OK)      \
    { /*ROS_INFO("\nERROR: %d\n", errcode);*/ \
    }

// const std::string cred_("\033[0;31m");
// const std::string creset_("\033[0m");
// const std::string cblue_("\033[0;34m");
// const std::string cgreen_("\033[0;32m");
// const std::string cyellow_("\033[0;33m");

const double SAMPLE_RATE = 1000; // Hz

enum SLOT_TIME
{
    NONE = 0,
    DEFAULT = 10
};

struct SLOTATTR
{
    uint chan;    // analog input channel
    uint tsettle; // settling time in microseconds
};

const SLOTATTR slotAttrs[16] = {
    {0, DEFAULT}, {1, DEFAULT}, {2, DEFAULT}, {3, DEFAULT}, {4, DEFAULT}, {5, DEFAULT}, {6, DEFAULT}, {7, DEFAULT}, {8, DEFAULT}, {9, DEFAULT}, {10, DEFAULT}, {11, DEFAULT}, {12, DEFAULT}, {13, DEFAULT}, {14, DEFAULT}, {15, DEFAULT}};

class sensoray826_dev
{

    static const int ADC_MAX_SLOT = 16;

    uint board; // change this if you want to use other than board number 0
    int errcode;
    int boardflags; // open 826 driver and find all 826 boards
    bool isOpen;

    bool isADCThreadOn;
    // boost::thread adcThread;

    uint _timeStamp[ADC_MAX_SLOT];
    int _adBuf[ADC_MAX_SLOT];

    enum AD_INDEX
    {
        LEFT_FOOT = 8,
        RIGHT_FOOT = 0
    };

public:
    // Analog Datas
    int adcDatas[ADC_MAX_SLOT];
    double adcVoltages[ADC_MAX_SLOT];
    double adcVoltagesPrev[ADC_MAX_SLOT];
    int burstNum[ADC_MAX_SLOT];

    const double calibrationMatrixLFoot[6][6] =
        {
            {7.20619, -0.25887, -3.85310, 193.37127, 10.91866, -191.60465},
            {4.88154, -220.16877, -1.11757, 110.83988, -11.50700, 110.61787},
            {-235.75704, -15.48050, -235.33214, -5.97867, -241.11222, -12.32612},
            {0.15684, -3.03866, 7.90152, 1.81391, -7.87886, 1.08755},
            {-9.07820, -0.66112, 4.48816, -2.49905, 4.42455, 2.91318},
            {0.00913, 4.99650, -0.01382, 5.03627, -0.55808, 4.89442}};
    const double calibrationMatrixRFoot[6][6] =
        {
            {-3.92743, 0.34092, 17.49126, -192.70354, -13.73590, 190.19215},
            {-8.92095, 217.00800, 7.88475, -112.09013, 11.38814, -109.84029},
            {229.58964, 2.55390, 229.83769, 6.32248, 241.92547, 8.43114},
            {-0.17926, 3.05960, -7.60972, -1.81525, 7.80692, -1.29264},
            {9.00835, 0.14417, -4.77982, 2.54902, -4.23245, -2.82298},
            {0.20565, -4.89759, 0.36305, -5.00519, 0.46035, -4.94725}};

    double leftFootAxisData[6];
    double rightFootAxisData[6];
    double leftFootAxisData_prev[6];
    double rightFootAxisData_prev[6];

    double leftFootBias[6];
    double rightFootBias[6];

    bool isCalibration;
    double dCalibrationTime;

    double _calibLFTData[6];
    double _calibRFTData[6];
    int _calibTimeIndex;
    int _calibMaxIndex;

public:
    sensoray826_dev(int boardno) : board(boardno), errcode(S826_ERR_OK), isOpen(false), isADCThreadOn(false), isCalibration(false), dCalibrationTime(5.0) {}
    virtual ~sensoray826_dev()
    {
        analogSampleStop();
        S826_SystemClose();
    }

    int open()
    {
        boardflags = S826_SystemOpen();
        if (boardflags < 0)
        {
            ROS_ERROR("BOARD ERROR");
            errcode = boardflags; // problem during open
            return errcode;
        }
        else if ((boardflags & (1 << board)) == 0)
        {
            int i;
            ROS_ERROR("TARGET BOARD of index %d NOT FOUND\n", board); // driver didn't find board you want to use
            for (i = 0; i < 8; i++)
            {
                if (boardflags & (1 << i))
                {
                    ROS_WARN("board %d detected. try [ %d ] board \n", i, i);
                }
            }
            return 0;
        }
        else
        {
            for (int i = 0; i < 8; i++)
            {
                if (boardflags & (1 << i))
                {
                    printf("\033[0;32m    FT : board %d detected.\033[0m\n", i);
                }
            }
            isOpen = true;
            return 1;
        }

        switch (errcode)
        {
        case S826_ERR_OK:
            break;
        case S826_ERR_BOARD:
            ROS_ERROR("Illegal board number");
            break;
        case S826_ERR_VALUE:
            ROS_ERROR("Illegal argument");
            break;
        case S826_ERR_NOTREADY:
            ROS_ERROR("Device not ready or timeout");
            break;
        case S826_ERR_CANCELLED:
            ROS_ERROR("Wait cancelled");
            break;
        case S826_ERR_DRIVER:
            ROS_ERROR("Driver call failed");
            break;
        case S826_ERR_MISSEDTRIG:
            ROS_ERROR("Missed adc trigger");
            break;
        case S826_ERR_DUPADDR:
            ROS_ERROR("Two boards have same number");
            break;
            S826_SafeWrenWrite(board, 0x02);
        case S826_ERR_BOARDCLOSED:
            ROS_ERROR("Board not open");
            break;
        case S826_ERR_CREATEMUTEX:
            ROS_ERROR("Can't create mutex");
            break;
        case S826_ERR_MEMORYMAP:
            ROS_ERROR("Can't map board");
            break;
        default:
            ROS_ERROR("Unknown error");
            break;
        }
    }

    /**
     * @brief analogSingleSamplePrepare, Preparing ADC but not turning on the AD thread
     * @param slotattrs ADC channel slots
     * @param count number of adc channel
     */
    void analogSingleSamplePrepare(const SLOTATTR *slotattrs, int count)
    {
        for (int i = 0; i < count; i++)
        {
            PRINT_ERR(S826_AdcSlotConfigWrite(board, i, slotattrs[i].chan, slotattrs[i].tsettle, S826_ADC_GAIN_1));
        }
        PRINT_ERR(S826_AdcSlotlistWrite(board, 0xFFFF, S826_BITWRITE)); // enable all timeslots
        PRINT_ERR(S826_AdcTrigModeWrite(board, 0));                     // select continuous (untriggered) mode
        PRINT_ERR(S826_AdcEnableWrite(board, 1));                       // enable conversions
    }

    /**
     * @brief analogSampleStop Stop current AD thread and ADC
     */
    void analogSampleStop()
    {
        if (isADCThreadOn)
        {
            isADCThreadOn = false;
            //   adcThread.join();
        }
        PRINT_ERR(S826_AdcEnableWrite(board, 0)); // halt adc conversions
    }
    /**
     * @brief analogOversample Do a single sample
     * @warning before calling this method, you should check whether ADC is prepared.
     */
    void analogOversample()
    {
        uint slotList = 0xFFFF;
        PRINT_ERR(S826_AdcRead(board, _adBuf, _timeStamp, &slotList, 0));

        for (int i = 0; i < ADC_MAX_SLOT; i++)
        {
            if ((((slotList >> (int)i) & 1) != 0))
            {
                // extract adcdata, burstnum, and bufoverflowflag from buf
                adcDatas[i] = (int16_t)((_adBuf[i] & 0xFFFF));
                burstNum[i] = ((uint32_t)_adBuf[i] >> 24);
                adcVoltages[i] = adcDatas[i] * 10.0 / 32768;
            }
        }
       // std::cout<<"1 :"<< adcVoltages[0] << " 2 : " << adcVoltages[1] << " 3 : " << adcVoltages[2] << " 4 : " << adcVoltages[3] << " 5 : " << adcVoltages[4] << " 6 : " << adcVoltages[5] << " 7 : " << 0.0 << " 8 : " << 0.0 << " 9 : " << adcVoltages[8] << " 10 : " << adcVoltages[9] << " 11 : " << adcVoltages[10] << " 12 : " << adcVoltages[11]<< " 13 : " << adcVoltages[12]<< " 14 : " << adcVoltages[13] << " 15 : " << adcVoltages[14] << " 16 : " << adcVoltages[15]    <<std::endl;
    }

    double lowPassFilter(double input, double prev, double ts, double tau)
    {
        return (tau * prev + ts * input) / (tau + ts);
    }

    void initCalibration()
    {
        for (int i = 0; i < 6; i++)
        {
            _calibLFTData[i] = 0.0;
            _calibRFTData[i] = 0.0;
            leftFootAxisData_prev[i] = 0.0;
            rightFootAxisData_prev[i] = 0.0;
        }
        for (int i = 0; i < 6; i++)
        {
            leftFootBias[i] = _calibLFTData[i];
            rightFootBias[i] = _calibRFTData[i];
        }
        _calibMaxIndex = dCalibrationTime * SAMPLE_RATE;
        printf("    FT : calibration Initialize... time = %.1lf sec, total %d samples \n", dCalibrationTime, _calibMaxIndex);
    }

    void calibrationFTData(bool ft_calib_finish)
    {
        if (ft_calib_finish == false)
        {
            for (int i = 0; i < 6; i++)
            {
                double _lf = 0.0;
                double _rf = 0.0;
                for (int j = 0; j < 6; j++)
                {
                    _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                    _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
                }
                _calibLFTData[i] += _lf / _calibMaxIndex;
                _calibRFTData[i] += _rf / _calibMaxIndex;
            }
        }
        else
        {
            for (int i = 0; i < 6; i++)
            {
                leftFootBias[i] = _calibLFTData[i];
                rightFootBias[i] = _calibRFTData[i];
            }
        }
    }

    void computeFTData(bool ft_calib_finish)
    {
        for (int i = 0; i < 6; i++)
        {
            double _lf = 0.0;
            double _rf = 0.0;
            for (int j = 0; j < 6; j++)
            {
                _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
            }

            _lf -= leftFootBias[i];
            _rf -= rightFootBias[i];

            leftFootAxisData[i] = _lf;  //lowPassFilter(_lf, leftFootAxisData_prev[i], 1.0 / SAMPLE_RATE, 0.05);
            rightFootAxisData[i] = _rf; //lowPassFilter(_rf, rightFootAxisData_prev[i], 1.0/ SAMPLE_RATE,0.05);

            //    leftFootAxisData_prev[i] = leftFootAxisData[i];
            //    rightFootAxisData_prev[i] = rightFootAxisData[i];
        }
        //std::cout << "FxL " << leftFootAxisData[0]<< " FzL " << leftFootAxisData[2] << " FxR " <<rightFootAxisData[0]<< " FzR" << rightFootAxisData[2] << std::endl;
    }
};
