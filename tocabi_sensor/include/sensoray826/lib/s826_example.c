///////////////////////////////////////////////////////////////////////////////////////////////
// SENSORAY MODEL 826 PROGRAMMING EXAMPLES
// This file contains simple functions that show how to program the 826.
// Copyright (C) 2012 Sensoray
///////////////////////////////////////////////////////////////////////////////////////////////

#ifndef _LINUX
#include <windows.h>
#include <conio.h>
#endif

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#ifndef _LINUX
#include "..\826api.h"
#else
#include "826api.h"
#endif


// Helpful macros for DIOs
#define DIO(C)                  ((uint64)1 << (C))                          // convert dio channel number to uint64 bit mask
#define DIOMASK(N)              {(uint)(N) & 0xFFFFFF, (uint)((N) >> 24)}   // convert uint64 bit mask to uint[2] array
#define DIOSTATE(STATES,CHAN)   ((STATES[CHAN / 24] >> (CHAN % 24)) & 1)    // extract dio channel's boolean state from uint[2] array


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ERROR HANDLING
// These examples employ very simple error handling: if an error is detected, the example functions will immediately return an error code.
// This behavior may not be suitable for some real-world applications but it makes the code easier to read and understand. In a real
// application, it's likely that additional actions would need to be performed. The examples use the following X826 macro to handle API
// function errors; it calls an API function and stores the returned value in errcode, then returns immediately if an error was detected.

#define X826(FUNC)   if ((errcode = FUNC) != S826_ERR_OK) { printf("\nERROR: %d\n", errcode); return errcode;}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Counter utility functions used by the demos.

// PERIODIC TIMER --------------------

// Counter mode: count down at 1MHz, preload upon start or when 0 reached, assert ExtOut when not 0
#define TMR_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_OM_NOTZERO)

// Configure a counter channel to operate as a periodic timer and start it running.
static int PeriodicTimerStart(uint board, uint counter, uint period)
{
    int errcode;
    X826( S826_CounterStateWrite(board, counter, 0)                         );     // halt channel if it's running
    X826( S826_CounterModeWrite(board, counter, TMR_MODE)                   );     // configure counter as periodic timer
    X826( S826_CounterSnapshotConfigWrite(board, counter, 4, S826_BITWRITE) );     // capture snapshots at zero counts
    X826( S826_CounterPreloadWrite(board, counter, 0, period)               );     // timer period in microseconds
    X826( S826_CounterStateWrite(board, counter, 1)                         );     // start timer
    return errcode;
}

// Halt channel operating as periodic timer.
static int PeriodicTimerStop(uint board, uint counter)
{
    return S826_CounterStateWrite(board, counter, 0);
}

// Wait for periodic timer event.
static int PeriodicTimerWait(uint board, uint counter, uint *timestamp) 
{
    uint counts, tstamp, reason;    // counter snapshot
    int errcode = S826_CounterSnapshotRead(board, counter, &counts, &tstamp, &reason, S826_WAIT_INFINITE);    // wait for timer snapshot
    if (timestamp != NULL)
        *timestamp = tstamp;
    return errcode;
}

// PWM GENERATOR -----------------------

// Counter mode: count down at 1MHz, preload when 0 reached, use both preload registers, assert ExtOut when Preload1 is active
#define PWM_MODE  (S826_CM_K_1MHZ | S826_CM_UD_REVERSE | S826_CM_PX_ZERO | S826_CM_PX_START | S826_CM_BP_BOTH | S826_CM_OM_PRELOAD)

//#define PWM_MODE 0x01682020

// Configure a counter channel to operate as a pwm generator and start it running.
static int PwmGeneratorStart(uint board, uint counter, uint ontime, uint offtime)
{
    int errcode;
    X826( S826_CounterStateWrite(board, counter, 0)                         );  // halt counter channel if it's running
    X826( S826_CounterModeWrite(board, counter, PWM_MODE)                   );  // configure counter as pwm generator
    X826( S826_CounterSnapshotConfigWrite(board, counter, 0, S826_BITWRITE) );  // don't need counter snapshots -- we're just outputting pwm signal
    X826( S826_CounterPreloadWrite(board, counter, 0, offtime)              );  // program pwm on-time in microseconds
    X826( S826_CounterPreloadWrite(board, counter, 1, ontime)               );  // program pwm off-time in microseconds
    X826( S826_CounterStateWrite(board, counter, 1)                         );  // start pwm generator
    return errcode;
}

// Halt channel operating as pwm generator.
static int PwmGeneratorStop(uint board, uint counter)
{
    return S826_CounterStateWrite(board, counter, 0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEMO: SIMPLE PERIODIC TIMER
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int DemoPeriodicTimer(uint board)
{
    uint counter    = 0;            // counter channel to use as a periodic timer (0 to 5)
    uint period     = 500000;       // timer interval in microseconds (500000 = 0.5 seconds)

    int i;
    int errcode;
    uint tstamp;    // counter snapshot's timestamp
    uint counts;

    printf("\nDemoPeriodicTimer\n");

    X826( PeriodicTimerStart(board, counter, period) );     // configure counter as periodic timer and start it running.

    for (i = 0; i < 10; i++)                                // repeat a few times ...
    {   
        X826( PeriodicTimerWait(board, counter, &tstamp) );     // wait for timer snapshot
        X826( S826_CounterRead(board, counter, &counts) );      // get counts - just to exercise CounterRead
        printf("timestamp:%u\tcounts:%u\n", (uint) tstamp, counts); // report timestamp
    }

    X826( PeriodicTimerStop(board, counter) );              // halt timer
    return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEMO: DIO EDGE DETECTION USING PWM OUTPUT AS EDGE GENERATOR.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int DemoDioEdgeDetect(uint board)
{
    // Configuration constants.
    uint diochan    = 21;                        // detect edges on this dio channel (0 to 47)

    int i;
    int errcode;
    int rc;
    uint diostates[2];
    uint diomask[]  = DIOMASK( DIO(diochan) );  // bit mask for edge detecting dio
    uint counter    = 5;              // use this counter channel as pwm generator; this depends on the dio channel as each dio is routeable to a specific counter's ExtOut

    printf("\nDemoDioEdgeDetect\n");

    // enable writing to safe mode registers
    rc = S826_SafeWrenWrite(board, 0x02);
    if (rc != 0) {
        printf("failed to enable wren for watchdog\n");
        return rc;
    }    

    // Configure counter as pwm generator, route pwm signal to dio0, enable edge captures on dio0.	    
    X826( S826_DioOutputSourceWrite(board, diomask)                         );  // route counter's ExtOut signal to dio pin
    X826( S826_DioCapEnablesWrite(board, diomask, diomask, S826_BITWRITE)   );  // enable both rising and falling edge capturing on dio0 pin		
    X826( S826_DioCapRead(board, diomask, 0, 0)                             );  // reset dio capture flag if it's already set; note that this can modify diomask
    X826( PwmGeneratorStart(board, counter, 300000, 700000)                 );  // start pwm generator with 300ms/700ms on/off times

    for (i = 0; i <= 20; i++)                                                   // Loop for 20 edge events ...
    {
        uint chanlist[] = DIOMASK( DIO(diochan) );               
        X826( S826_DioCapRead(board, chanlist, 0, S826_WAIT_INFINITE)       );  //   wait for dio0 edge event
        X826( S826_DioInputRead(board, diostates)                           );  //   read dio pin states
        printf("%d ", DIOSTATE(diostates, diochan));                            //   report dio0 pin state	
	fflush(stdout);								//   output each iteration instead of waiting for the newline
    }
    printf("\n");

    X826( PwmGeneratorStop(board, counter)                                  );  // halt pwm generator
    return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEMO: GENERATE SINE WAVE OUTPUT
// Resources: Counter channel (used as periodic timer), analog output channel.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#define PI  3.1415926535

static int DemoSinewaveGenerator(uint board)
{
    // Configuration values
    double freq     = 50.0;         // sine frequency in Hz
    uint amplitude  = 32767;        // peak amplitude (32767=10V). Must be < 32768
    uint tsample    = 250;          // sample time in microseconds. Must be fast enough to meet nyquist
    double duration = 5.0;          // waveform duration in seconds
    uint aout       = 0;            // output sine wave on thiS826_WAIT_INFINITEs dac channel
    uint counter    = 0;            // use this counter channel as dac sample timer

    uint tstamp;                    // snapshot timestamp
    uint tbegin;                    // timestamp at t0
    double runtime;                 // runtime in seconds
    uint dacval;                    // dac setpoint
    int errcode;

    printf("\nDemoSinewaveGenerator\n");

    X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_10_10, 0)   );      // program dac output range: -10V to +10V
    X826( PeriodicTimerStart(board, counter, tsample)               );      // configure counter0 as periodic timer and start it running
    X826( PeriodicTimerWait(board, counter, &tstamp)                );      // wait for first sampluint diomask[]  = DIOMASK( DIO(diochan) );e time (t0)
    tbegin = tstamp;                                                        // get t0 timestamp

    do {                                                                    // repeat until waveform duration has elapsed:

        runtime = (double)(tstamp - tbegin) / 1000000.0;                        // compute time elapsed since t0
        dacval  = (uint)(32768 + amplitude * sin(2.0 * PI * freq * runtime));   // compute sinewave amplitude based on elapsed time

        X826( S826_DacDataWrite(board, aout, dacval, 0)  );         // output next sinewave sample
        errcode = PeriodicTimerWait(board, counter, &tstamp);         // wait for next sample time
        if (errcode == -15) {
            printf("overflow, make thread priority higher and/or use low latency kernel\n");
        } else if (errcode != 0) {
            printf("\nERROR: %d\n", errcode); return errcode;
        }
    }
    while (runtime < duration);

    X826( PeriodicTimerStop(board, counter));                               // halt periodic timer
    printf("\nDemoSinewaveGenerator complete[%d]\n", errcode);
    return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEMO: OVERSAMPLE TWO ANALOG INPUTS
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static int DemoAnalogOversample(uint board)
{
    // Adc timeslot attributes.
    // In each adc burst, convert analog channel 0 eight times then channel 1 eight times. Allow each channel to settle for 50 microseconds
    // after switching (before first conversion); subsequent conversions on the same channel don't need additional settling time.
    struct SLOTATTR {
        uint chan;      // analog input channel
        uint tsettle;   // settling time in microseconds
    } attr[16] = {                                                          // during each adc burst:
        {0, 50}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0}, {0, 0},    //   switch to channel 0, wait 50us, digitize eight times
        {1, 50}, {1, 0}, {1, 0}, {1, 0}, {1, 0}, {1, 0}, {1, 0}, {1, 0}};   //   switch to channel 1, wait 50us, digitize eight times


    int i;
    int slot;
    uint slotlist;
    int adcbuf[16];
    int errcode;    

    printf("\nDemoAnalogOversample\n");

    // Program attributes for all timeslots.
    for (slot = 0; slot < 16; slot++)
        X826( S826_AdcSlotConfigWrite(board, slot, attr[slot].chan, attr[slot].tsettle, S826_ADC_GAIN_1) );

    // Configure adc system and start it running.
    X826( S826_AdcSlotlistWrite(board, 0xFFFF, S826_BITWRITE)   );  // enable all timeslots
    X826( S826_AdcTrigModeWrite(board, 0)                       );  // select continuous (untriggered) mode
    X826( S826_AdcEnableWrite(board, 1)                         );  // enable conversions

    // Fetch and display oversampled data.
    for (i = 0; i < 10; i++)                                        // repeat a few times:
    {
        int sum[] = {0, 0};                                         // zero the adc sample accumulators, one per analog input channel
        slotlist = 0xFFFF;
        X826( S826_AdcRead(board, adcbuf, NULL, &slotlist, 1000) ); // read adc data from 16 slots

        for (slot = 0; slot < 16; slot++)                           // sum the 8 samples from each analog input 
            sum[slot >> 3] += (short)( adcbuf[slot] & 0xFFFF );		// masking off sample meta-data
		
        printf("CH0=%3.3fV, CH1=%3.3fV\n", (double)(sum[0]*10)/(8*32768), (double)(sum[1]*10)/(8*32768));     // compute average values, report measured voltages
    }

    X826( S826_AdcEnableWrite(board, 0)                         );  // halt adc conversions
    return errcode;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// DEMO: DUPLICATE ANALOG INPUT WAVEFORM ON ANALOG OUTPUT
// Resources: Analog input channel, analog output channel, counter channel.
// This demo periodically samples an analog input (via counter-driven hardware trigger) and reproduces it on an analog output.
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// Helper: Read data from one slot
static int AdcReadSlot(uint board, uint slot, int *slotdata)
{
    int adcbuf[16];
    uint slotlist = 1 << slot;                                                      // set up the adc slot list; we are only interested in one slot
    int errcode = S826_AdcRead(board, adcbuf, NULL, &slotlist, 1000);                     // wait for data to arrive on the slot of interest (in response to adc hardware trigger)
    if ((unsigned int) errcode == S826_ERR_MISSEDTRIG)      // this application doesn't care about adc missed triggers
        errcode = S826_ERR_OK;
    *slotdata = adcbuf[slot] & 0xFFFF;                                                       // copy adc data to slotdata buffer

    return errcode;
}

// The demo
static int DemoAnalogTrack(uint board)
{
    // Configuration constants.
    uint ain        = 0;        // analog input channel to track
    uint aout       = 0;        // output duplicate waveform on this dac channel
    uint slot       = 0;        // use this adc timeslot (the "slot of interest") to sample the analog input
    uint tsample    = 100;      // sampling interval in microseconds
    uint duration   = 10;       // track for this many seconds
    uint counter    = 0;        // use this counter channel to periodically trigger adc conversions

    int slotdata;               // adc data from the slot of interest
    int errcode;
    uint i;

    printf("\nDemoAnalogTrack\n");

    duration *= (1000000 / tsample);            // convert tracking time units from seconds to equivalent sampling ticks
    
    // Configure interfaces and start them running.
    X826( S826_DacRangeWrite(board, aout, S826_DAC_SPAN_10_10, 0)       );  // program dac output range: -10V to +10V
    X826( S826_AdcSlotConfigWrite(board, slot, ain, 0, S826_ADC_GAIN_1) );  // program adc timeslot attributes: slot, chan, 0us settling time
    X826( S826_AdcSlotlistWrite(board, 1 << slot, S826_BITWRITE)        );  // enable adc timeslot; disable all other slots
    X826( S826_AdcTrigModeWrite(board, 0x80 | (48 + counter))           );  // enable adc hardware triggering, use counter channel's ExtOut as trigger
    X826( S826_AdcEnableWrite(board, 1)                                 );  // enable adc conversions
    X826( PeriodicTimerStart(board, counter, tsample)                   );  // start periodic sample timer
    X826( AdcReadSlot(board, slot, &slotdata)                           );  // discard first adc data sample (because it might need some settling time)
		
    for (i = 0; i < duration; i++)                                          // repeat until tracking time elapses:
    {	
        X826( AdcReadSlot(board, slot, &slotdata)                   );          // wait for adc data sample
        X826( S826_DacDataWrite(board, aout, (short)slotdata + 0x8000, 0)  );   // output dac sample - cast to short to get sign
    }

    X826( S826_AdcEnableWrite(board, 0)                                 );  // halt adc conversions
    X826( PeriodicTimerStop(board, counter)                             );  // halt periodic timer
    return errcode;
}





// The demo
static int TestDacRW(uint board)
{
    uint safemode = 0;
    uint setpoint;
    uint range;
    uint chan = 0;
    int errcode;
    uint i;

    printf("\nTestDacRW\n");

    

    X826( S826_SafeWrenWrite(board, safemode) );
    X826( S826_DacRangeWrite(board, chan, S826_DAC_SPAN_5_5, safemode)       );  // program dac output range: -10V to +10V

    for (i = 0; i < 0xFFFF; i++)
    {
        X826( S826_DacDataWrite(board, chan, i, safemode) );
        X826( S826_DacRead(board, chan, &range, &setpoint, safemode) );
        if (i != setpoint)
            printf("%d != %d\n", i, setpoint);

    }


    return errcode;
}











#if 0
DWORD WINAPI testThread(
  _In_  LPVOID lpParameter
)
{
    uint board = (uint) lpParameter;
    Sleep(1);
    S826_WatchdogWaitCancel(board);
}
#endif

// The demo.
static int DemoWatchdog(uint board)
{
    int rc;
    uint timing[5];
    // set timer 1 time-out to around 1 second
    timing[0] = 50000000;
    printf("\nDemoWatchdog\n");
    // enable writing to watchdog
    rc = S826_SafeWrenWrite(board, 0x02);
    if (rc != 0) {
        printf("failed to enable wren for watchdog\n");
        return rc;
    }
    rc = S826_WatchdogConfigWrite(board, 0x00, timing);
    if (rc != 0) {
        printf("failed to configure WD\n");
        return rc;
    }

    rc = S826_WatchdogEnableWrite(board, 1);
    if (rc != 0) {
        printf("failed to enable WD\n");
        return rc;
    }
    
    // commented out.  for testing watchdog cancel
    // CreateThread(NULL, 4096, testThread, board, 0, NULL);

    // watch indefinitely for watchdog
    rc = S826_WatchdogEventWait(board, S826_WAIT_INFINITE);

    switch (rc) {
        case 0:
            printf("watchdog successfully expired\n");
            break;
        case S826_ERR_NOTREADY:
            printf("WD wait timed out\n");
            break;
        case S826_ERR_CANCELLED:
            printf("WD wait cancelled\n");
            break;
    default:
        printf("error %d\n", rc);
        
    }
    // disable watchdog
    rc = S826_WatchdogEnableWrite(board, 0);
    if (rc != 0) {
        printf("failed to disable WD\n");
        return rc;
    }
    return 0;
}
 



////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Main function.

int main(int argc, char **argv)
{
    uint board      = 0;                        // change this if you want to use other than board number 0
    int errcode     = S826_ERR_OK;	
    int boardflags  = S826_SystemOpen();        // open 826 driver and find all 826 boards
    if (argc > 1)
        board = atoi(argv[1]);

    if (boardflags < 0)
        errcode = boardflags;                       // problem during open
    else if ((boardflags & (1 << board)) == 0) {
        int i;
        printf("TARGET BOARD of index %d NOT FOUND\n",board);         // driver didn't find board you want to use
        for (i = 0; i < 8; i++) {
            if (boardflags & (1 << i)) {
                printf("board %d detected. try \"./s826demo %d\"\n", i, i);
            }
        }
    } else  {
        // Execute the demo functions. Uncomment any functions you want to run.
        X826( DemoPeriodicTimer(board)      );      // periodic timer
        X826( DemoDioEdgeDetect(board)      );      // dio edge detection
        X826( DemoSinewaveGenerator(board)  );      // analog sinewave output
        X826( TestDacRW(board) );
        X826( DemoAnalogOversample(board)   );      // oversampled analog inputs
        X826( DemoAnalogTrack(board)        );      // copy analog input to dac
        X826( DemoWatchdog(board)           );      // watchdog timer
    }

    switch (errcode)
    {
    case S826_ERR_OK:           break;
    case S826_ERR_BOARD:        printf("Illegal board number"); break;
    case S826_ERR_VALUE:        printf("Illegal argument"); break;
    case S826_ERR_NOTREADY:     printf("Device not ready or timeout"); break;
    case S826_ERR_CANCELLED:    printf("Wait cancelled"); break;
    case S826_ERR_DRIVER:       printf("Driver call failed"); break;
    case S826_ERR_MISSEDTRIG:   printf("Missed adc trigger"); break;
    case S826_ERR_DUPADDR:      printf("Two boards have same number"); break;S826_SafeWrenWrite(board, 0x02);
    case S826_ERR_BOARDCLOSED:  printf("Board not open"); break;
    case S826_ERR_CREATEMUTEX:  printf("Can't create mutex"); break;
    case S826_ERR_MEMORYMAP:    printf("Can't map board"); break;
    default:                    printf("Unknown error"); break;
    }

    
#ifndef _LINUX	
    printf("\nKeypress to exit ...\n\n");
    while (!_kbhit());
    _getch();
#endif

    S826_SystemClose();
    return 0;
}

