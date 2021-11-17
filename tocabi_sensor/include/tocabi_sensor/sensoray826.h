#include "lib/826api.h"
#include "math_type_define.h"
#include <ros/ros.h>
#include <thread>

#define PRINT_ERR(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) {/*ROS_INFO("\nERROR: %d\n", errcode);*/}

const double SAMPLE_RATE = 1000; // Hz

enum SLOT_TIME {NONE = 0, DEFAULT = 10};

struct SLOTATTR
{
    uint chan;      // analog input channel
    uint tsettle;   // settling time in microseconds
};

const SLOTATTR slotAttrs[16] = { 
    {0, DEFAULT}, {1, DEFAULT}, {2, DEFAULT}, {3, DEFAULT},
    {4, DEFAULT}, {5, DEFAULT}, {6, DEFAULT}, {7, DEFAULT},
    {8, DEFAULT}, {9, DEFAULT}, {10, DEFAULT}, {11, DEFAULT},
    {12, DEFAULT}, {13, DEFAULT}, {14, DEFAULT}, {15, DEFAULT}
};

class sensoray826_dev
{

    static const int ADC_MAX_SLOT = 16;

    uint board;// change this if you want to use other than board number 0
    int errcode;
    int boardflags;        // open 826 driver and find all 826 boards
    bool isOpen;

    bool isADCThreadOn;
   // boost::thread adcThread;


    uint _timeStamp[ADC_MAX_SLOT];
    int _adBuf[ADC_MAX_SLOT];

    enum AD_INDEX {LEFT_FOOT = 8, RIGHT_FOOT = 0};


public:
    // Analog Datas
    int adcDatas[ADC_MAX_SLOT];
    double adcVoltages[ADC_MAX_SLOT];
    double adcVoltagesPrev[ADC_MAX_SLOT];
    int burstNum[ADC_MAX_SLOT];

    const double calibrationMatrixLFoot[6][6] = 
    {
    {-7.31079,   0.57154,   3.96660, -193.51164, -10.48427,  191.62950},
    {-4.75952,  219.70659,   0.37703, -110.68639,  11.48251, -110.46257},
    {235.50673,  15.29352,  235.02213,   6.13158,  241.08552,  12.23876},
    {-0.11263,   3.04924,  -7.88257,  -1.87193,   7.90115,  -1.05469},
    {9.07350,   0.69075,  -4.45980,   2.47521,  -4.43928,  -2.91897},
    {0.06164,  -4.92947,   0.06684,  -5.01570,   0.62875,  -4.91040}
    };
    const double calibrationMatrixRFoot[6][6] = 
    {
    {-3.92743,   0.34092,  17.49126, -192.70354, -13.73590,  190.19215},
    {-8.92095,  217.00800,   7.88475, -112.09013,  11.38814, -109.84029},
    {229.58964,  2.55390,  229.83769,   6.32248,  241.92547,   8.43114},
    {-0.17926,   3.05960,  -7.60972,  -1.81525,   7.80692,  -1.29264},
    {9.00835,   0.14417,  -4.77982,   2.54902,  -4.23245,  -2.82298},
    {0.20565,  -4.89759,   0.36305,  -5.00519,   0.46035,  -4.94725}
    };

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
    virtual ~sensoray826_dev() { analogSampleStop(); S826_SystemClose(); }

    int open()
    {
        boardflags = S826_SystemOpen();
        if (boardflags < 0)
            {
                ROS_ERROR("BOARD ERROR"); 
                errcode = boardflags; // problem during open
                return errcode;
            }
        else if ((boardflags & (1 << board)) == 0) {
            int i;
            ROS_ERROR("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
            for (i = 0; i < 8; i++) {
                if (boardflags & (1 << i)) {
                    ROS_WARN("board %d detected. try [ %d ] board \n", i, i);
                }
            }
            return 0;
        }
        else
        {
            for (int i = 0; i < 8; i++) {
                if (boardflags & (1 << i)) {
                    ROS_INFO("board %d detected." , i);
                }
            }
            isOpen = true;
            return 1;
        }

        switch (errcode)
        {
            case S826_ERR_OK:           break;
            case S826_ERR_BOARD:        ROS_ERROR("Illegal board number"); break;
            case S826_ERR_VALUE:        ROS_ERROR("Illegal argument"); break;
            case S826_ERR_NOTREADY:     ROS_ERROR("Device not ready or timeout"); break;
            case S826_ERR_CANCELLED:    ROS_ERROR("Wait cancelled"); break;
            case S826_ERR_DRIVER:       ROS_ERROR("Driver call failed"); break;
            case S826_ERR_MISSEDTRIG:   ROS_ERROR("Missed adc trigger"); break;
            case S826_ERR_DUPADDR:      ROS_ERROR("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
            case S826_ERR_BOARDCLOSED:  ROS_ERROR("Board not open"); break;
            case S826_ERR_CREATEMUTEX:  ROS_ERROR("Can't create mutex"); break;
            case S826_ERR_MEMORYMAP:    ROS_ERROR("Can't map board"); break;
            default:                    ROS_ERROR("Unknown error"); break;
        }
    }

    /**
     * @brief analogSingleSamplePrepare, Preparing ADC but not turning on the AD thread
     * @param slotattrs ADC channel slots
     * @param count number of adc channel
     */
    void analogSingleSamplePrepare(const SLOTATTR *slotattrs , int count)
    {
        for(int i=0; i<count; i++)
        {
            PRINT_ERR(S826_AdcSlotConfigWrite(board, i, slotattrs[i].chan, slotattrs[i].tsettle, S826_ADC_GAIN_1) );
        }
        PRINT_ERR( S826_AdcSlotlistWrite(board, 0xFFFF, S826_BITWRITE)   );  // enable all timeslots
        PRINT_ERR( S826_AdcTrigModeWrite(board, 0)                       );  // select continuous (untriggered) mode
        PRINT_ERR( S826_AdcEnableWrite(board, 1)                         );  // enable conversions  
    }

    /**
     * @brief analogSampleStop Stop current AD thread and ADC
     */
    void analogSampleStop()
    {
        if(isADCThreadOn)
        {
            isADCThreadOn = false;
         //   adcThread.join();
        }
        PRINT_ERR( S826_AdcEnableWrite(board, 0)                         );  // halt adc conversions
    }
    /**
     * @brief analogOversample Do a single sample
     * @warning before calling this method, you should check whether ADC is prepared.
     */
    void analogOversample()
    {
        uint slotList = 0xFFFF;
        PRINT_ERR ( S826_AdcRead(board, _adBuf, _timeStamp, &slotList, 0));         
  
        for(int i=0; i<ADC_MAX_SLOT; i++)
        {
            if ((((slotList >> (int)i) & 1) != 0)) {
                // extract adcdata, burstnum, and bufoverflowflag from buf
                adcDatas[i] = (int16_t)((_adBuf[i] & 0xFFFF));
                burstNum[i] = ((uint32_t)_adBuf[i] >> 24);
                adcVoltages[i] = adcDatas[i] * 10.0 / 32768;
            }
         //   std::cout<<"1 :"<< adcVoltages[0] << " 2 : " << adcVoltages[1] << " 3 : " << adcVoltages[2] << " 4 : " << adcVoltages[3] << " 5 : " << adcVoltages[4] << " 6 : " << adcVoltages[5] << " 7 : " << adcVoltages[6] << " 8 : " << adcVoltages[7] << " 9 : " << adcVoltages[8] << " 10 : " << adcVoltages[9] << " 11 : " << adcVoltages[10] << " 12 : " << adcVoltages[11]<< " 13 : " << adcVoltages[12]<< " 14 : " << adcVoltages[13] << " 15 : " << adcVoltages[14] << " 16 : " << adcVoltages[15]    <<std::endl;
        }
    }

    double lowPassFilter(double input, double prev, double ts, double tau)
    {
        return (tau*prev + ts*input)/(tau+ts);
    }

    void initCalibration()
    {
        for(int i=0; i<6; i++)
        {
            _calibLFTData[i] = 0.0;
            _calibRFTData[i] = 0.0;
            leftFootAxisData_prev[i] = 0.0;
            rightFootAxisData_prev[i] = 0.0;
        }
        for(int i=0; i<6; i++)
        {
            leftFootBias[i] = _calibLFTData[i];
            rightFootBias[i] = _calibRFTData[i];
        }
        _calibMaxIndex = dCalibrationTime * SAMPLE_RATE;
        ROS_INFO("FT sensor calibration Initialize... time = %.1lf sec, total %d samples ", dCalibrationTime, _calibMaxIndex);
    }
  
    void calibrationFTData(bool ft_calib_finish)
    {
        if(ft_calib_finish == false)
        {
            for(int i=0; i<6; i++)
            {
                double _lf = 0.0;
                double _rf = 0.0;
                for(int j=0; j<6; j++)
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
            for(int i=0; i<6; i++)
            {
                leftFootBias[i] = _calibLFTData[i];
                rightFootBias[i] = _calibRFTData[i];
            }
        }
    }

    void computeFTData(bool ft_calib_finish)
    {
        for(int i=0; i<6; i++)
        {
            double _lf = 0.0;
            double _rf = 0.0;
            for(int j=0; j<6; j++)
            {
                _lf += calibrationMatrixLFoot[i][j] * adcVoltages[j + LEFT_FOOT];
                _rf += calibrationMatrixRFoot[i][j] * adcVoltages[j + RIGHT_FOOT];
            }

            _lf -= leftFootBias[i];
            _rf -= rightFootBias[i];

            leftFootAxisData[i] = lowPassFilter(_lf, leftFootAxisData_prev[i], 1.0 / SAMPLE_RATE, 0.05);
            rightFootAxisData[i] = lowPassFilter(_rf, rightFootAxisData_prev[i], 1.0/ SAMPLE_RATE,0.05);
            
            leftFootAxisData_prev[i] = leftFootAxisData[i];
            rightFootAxisData_prev[i] = rightFootAxisData[i];
        }
    }
};
