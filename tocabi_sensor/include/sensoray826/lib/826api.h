////////////////////////////////////////////////////////////////////////////////////////
// File      : 826api.h
// Function  : API for Sensoray's model 826 multi-function I/O board
// Usage     : Include in C/C++ applications using model 826
// Copyright : (C) 2012-2017 Sensoray
////////////////////////////////////////////////////////////////////////////////////////

#ifndef _INC_826API_H_
#define _INC_826API_H_

#ifdef __cplusplus
extern "C" {
#endif

#define S826_MAX_BOARDS         16          // Maximum number of boards supported by driver and API

// Error codes
#define S826_ERR_OK             0           // No error
#define S826_ERR_BOARD          (unsigned int)(-1)        // Illegal board number
#define S826_ERR_VALUE          (unsigned int)(-2)        // Illegal argument value
#define S826_ERR_NOTREADY       (unsigned int)(-3)        // Device not ready or timeout waiting for device
#define S826_ERR_CANCELLED      (unsigned int)(-4)        // Wait cancelled
#define S826_ERR_DRIVER         (unsigned int)(-5)        // Driver call failed
#define S826_ERR_MISSEDTRIG     (unsigned int)(-6)        // Missed adc trigger
#define S826_ERR_DUPADDR        (unsigned int)(-9)        // Two boards set to same board number
#define S826_ERR_BOARDCLOSED    (unsigned int)(-10)       // Board is not open
#define S826_ERR_CREATEMUTEX    (unsigned int)(-11)       // Can't create mutex
#define S826_ERR_MEMORYMAP      (unsigned int)(-12)       // Can't map board to memory address
#define S826_ERR_MALLOC         (unsigned int)(-13)       // Can't allocate memory
#define S826_ERR_FIFOOVERFLOW   (unsigned int)(-15)       // Counter's snapshot fifo overflowed
#define S826_ERR_LOCALBUS       (unsigned int)(-16)       // Can't read local bus (register contains illegal value)
#define S826_ERR_OSSPECIFIC     (unsigned int)(-100)      // Port-specific error (base error number)

// Number of available channels.
#define S826_NUM_COUNT          6           // Counters
#define S826_NUM_ADC            16          // Analog inputs
#define S826_NUM_DAC            8           // Analog outputs
#define S826_NUM_DIO            48          // Digital I/Os

// Analog input range codes
#define S826_ADC_GAIN_1         0           // -10V to +10V
#define S826_ADC_GAIN_2         1           // -5V to +5V
#define S826_ADC_GAIN_5         2           // -2V to +2V
#define S826_ADC_GAIN_10        3           // -1V to +1V

// Analog output range codes
#define S826_DAC_SPAN_0_5       0           // 0 to +5V
#define S826_DAC_SPAN_0_10      1           // 0 to +10V
#define S826_DAC_SPAN_5_5       2           // -5V to +5V
#define S826_DAC_SPAN_10_10     3           // -10V to +10V

// Counter ExtIn routing source
#define S826_INSRC_DIO(N)       (N)         // DIO channel N (0:47)
#define S826_INSRC_EXTOUT(N)    ((N) + 48)  // Counter channel N (0:5) ExtOut
#define S826_INSRC_VDIO(N)      ((N) + 54)  // Virtual DIO channel N (0:5)

// Snapshot reason bit masks
#define S826_SSRMASK_QUADERR    (1 << 8)    // Quadrature error
#define S826_SSRMASK_SOFT       (1 << 7)    // Soft snapshot
#define S826_SSRMASK_EXTRISE    (1 << 6)    // ExtIn rising edge
#define S826_SSRMASK_EXTFALL    (1 << 5)    // ExtIn falling edge
#define S826_SSRMASK_IXRISE     (1 << 4)    // Index rising edge
#define S826_SSRMASK_IXFALL     (1 << 3)    // Index falling edge
#define S826_SSRMASK_ZERO       (1 << 2)    // Zero counts reached
#define S826_SSRMASK_MATCH1     (1 << 1)    // Compare1 register match
#define S826_SSRMASK_MATCH0     (1 << 0)    // Compare0 register match

// Snapshot enable bit masks
#define S826_SS_ALL             0x00000001
#define S826_SS_ONE             0x00010001
// Capture all snapshots (don't disable after first snapshot)
#define S826_SS_ALL_EXTRISE     (S826_SS_ALL << 6)    // ExtIn rising edge
#define S826_SS_ALL_EXTFALL     (S826_SS_ALL << 5)    // ExtIn falling edge
#define S826_SS_ALL_IXRISE      (S826_SS_ALL << 4)    // Index rising edge
#define S826_SS_ALL_IXFALL      (S826_SS_ALL << 3)    // Index falling edge
#define S826_SS_ALL_ZERO        (S826_SS_ALL << 2)    // Zero counts reached
#define S826_SS_ALL_MATCH1      (S826_SS_ALL << 1)    // Compare1 register match
#define S826_SS_ALL_MATCH0      (S826_SS_ALL << 0)    // Compare0 register match
// Capture one snapshot (disable upon first snapshot)
#define S826_SS_FIRST_EXTRISE   (S826_SS_ONE << 6)    // ExtIn rising edge
#define S826_SS_FIRST_EXTFALL   (S826_SS_ONE << 5)    // ExtIn falling edge
#define S826_SS_FIRST_IXRISE    (S826_SS_ONE << 4)    // Index rising edge
#define S826_SS_FIRST_IXFALL    (S826_SS_ONE << 3)    // Index falling edge
#define S826_SS_FIRST_ZERO      (S826_SS_ONE << 2)    // Zero counts reached
#define S826_SS_FIRST_MATCH1    (S826_SS_ONE << 1)    // Compare1 register match
#define S826_SS_FIRST_MATCH0    (S826_SS_ONE << 0)    // Compare0 register match

// ControlWrite/ControlRead bit masks
#define S826_CONFIG_XSF         (1 << 3)    // Enable DIO47 to set SAF
#define S826_CONFIG_SAF         (1 << 1)    // SafeMode active

// Watchdog configuration bit masks
#define S826_WD_GSN             (1 << 6)    // Assert NMI upon timer1 timeout
#define S826_WD_SEN             (1 << 4)    // Activate safemode upon timer0 timeout
#define S826_WD_NIE             (1 << 3)    // Connect timer1 output to dio routing matrix NMI net
#define S826_WD_PEN             (1 << 2)    // Enable RST output to pulse
#define S826_WD_OEN             (1 << 0)    // Connect RST generator to dio routing matrix RST net

// Array indices for watchdog timing parameters
#define S826_WD_DELAY0          0           // Timer0 interval (20 ns resolution)
#define S826_WD_DELAY1          1           // Timer1 interval (20 ns resolution)
#define S826_WD_DELAY2          2           // Timer2 interval (20 ns resolution)
#define S826_WD_PWIDTH          3           // RST pulse width (ignored if PEN=0)
#define S826_WD_PGAP            4           // Time gap between RST pulses (ignored if PEN=0)

// SAFEN bit masks
#define S826_SAFEN_SWE          (1 << 1)    // Set write enable for safemode registers
#define S826_SAFEN_SWD          (1 << 0)    // Clear write enable for safemode registers

// Register Write/Bitset/Bitclear modes
#define S826_BITWRITE           0           // Write all bits unconditionally
#define S826_BITCLR             1           // Clear designated bits; leave others unchanged
#define S826_BITSET             2           // Set designated bits; leave others unchanged

// Wait types
#define S826_WAIT_ALL			0           // Wait for all listed events
#define S826_WAIT_ANY			1           // Wait for any listed event

// Wait durations
#define S826_WAIT_INFINITE      0xFFFFFFFF  // Use this the tmax value on any blocking function that needs infinite wait time

// Counter mode register bit masks ------------------------------

                                            // ExtIn polarity
#define S826_CM_IP_NORMAL       (0 << 30)   //   pass-thru
#define S826_CM_IP_INVERT       (1 << 30)   //   invert
                                            // ExtIn function
#define S826_CM_IM_OFF          (0 << 28)   //   not used
#define S826_CM_IM_COUNTEN      (1 << 28)   //   count permissive
#define S826_CM_IM_PRELOADEN    (2 << 28)   //   preload permissive
                                            // Retriggerability
#define S826_CM_NR_RETRIG       (0 << 23)   //   enable preloading when counts not zero
#define S826_CM_NR_NORETRIG     (1 << 23)   //   disable preloading when counts not zero
                                            // Count direction
#define S826_CM_UD_NORMAL       (0 << 22)   //   count up
#define S826_CM_UD_REVERSE      (1 << 22)   //   count down
                                            // Preload usage
#define S826_CM_BP_SINGLE       (0 << 21)   //   use only Preload0
#define S826_CM_BP_BOTH         (1 << 21)   //   toggle between Preload0 and Preload1
                                            // ExtOut function
#define S826_CM_OM_OFF          (0 << 18)   //   always '0'
#define S826_CM_OM_MATCH        (1 << 18)   //   pulse upon compare0 or Compare1 match snapshot
#define S826_CM_OM_PRELOAD      (2 << 18)   //   active when Preload1 is selected
#define S826_CM_OM_NOTZERO      (3 << 18)   //   active when counts != zero
#define S826_CM_OM_ZERO         (4 << 18)   //   active when counts == zero
                                            // ExtOut polarity
#define S826_CM_OP_NORMAL       (0 << 17)   //   active high
#define S826_CM_OP_INVERT       (1 << 17)   //   active low
                                            // Preload triggers
#define S826_CM_PX_START        (1 << 24)   //   upon counter enabled
#define S826_CM_PX_IXHIGH       (1 << 16)   //   while Index active (holds counts at preload value)
#define S826_CM_PX_IXRISE       (1 << 15)   //   upon Index rising edge
#define S826_CM_PX_IXFALL       (1 << 14)   //   upon Index falling edge
#define S826_CM_PX_ZERO         (1 << 13)   //   upon zero counts reached
#define S826_CM_PX_MATCH1       (1 << 12)   //   upon Compare1 counts reached
#define S826_CM_PX_MATCH0       (1 << 11)   //   upon Compare0 counts reached
                                            // Count enable trigger
#define S826_CM_TE_STARTUP      (0 << 9)    //   upon counter enabled
#define S826_CM_TE_IXRISE       (1 << 9)    //   upon Index rising edge
#define S826_CM_TE_PRELOAD      (2 << 9)    //   upon preloading
                                            // Count disable trigger
#define S826_CM_TD_NEVER        (0 << 7)    //   upon counter disabled
#define S826_CM_TD_IXFALL       (1 << 7)    //   upon Index falling edge
#define S826_CM_TD_ZERO         (2 << 7)    //   upon zero counts reached
                                            // Clock mode
#define S826_CM_K_ARISE         (0 << 4)    //   single-phase, ClkA rising edge
#define S826_CM_K_AFALL         (1 << 4)    //   single-phase, ClkA falling edge
#define S826_CM_K_1MHZ          (2 << 4)    //   single-phase, 1 MHz internal clock
#define S826_CM_K_50MHZ         (3 << 4)    //   single-phase, 50 MHz internal clock
#define S826_CM_K_CASCADE       (4 << 4)    //   single-phase, cascade-out of adjacent channel
#define S826_CM_K_QUADX1        (5 << 4)    //   quadrature x1, ClkA and ClkB
#define S826_CM_K_QUADX2        (6 << 4)    //   quadrature x2, ClkA and ClkB
#define S826_CM_K_QUADX4        (7 << 4)    //   quadrature x4, ClkA and ClkB
                                            // Index input source
#define S826_CM_XS_EXTNORMAL    0           //   IX input, pass-thru
#define S826_CM_XS_EXTINVERT    1           //   IX input, inverted
#define S826_CM_XS_EXTOUT(CTR)  ((CTR) + 2) //   ExtOut of any counter (CTR in range 0..5)
#define S826_CM_XS_R1HZ         8           //   0.1 Hz internal tick generator
#define S826_CM_XS_1HZ          9           //   1 Hz internal tick generator
#define S826_CM_XS_10HZ         10          //   10 Hz internal tick generator
#define S826_CM_XS_100HZ        11          //   100 Hz internal tick generator
#define S826_CM_XS_1KHZ         12          //   1 kHz internal tick generator
#define S826_CM_XS_10KHZ        13          //   10 kHz internal tick generator
#define S826_CM_XS_100KHZ       14          //   100 kHz internal tick generator
#define S826_CM_XS_1MHZ         15          //   1 MHz internal tick generator

// Port-dependent defines needed for app builds ------------------

#ifndef S826_API
#ifdef _WIN32
#define S826_API
#define S826_CC __stdcall
typedef unsigned int uint;
typedef unsigned short uint16;
#else
#define S826_API
#define S826_CC
#ifndef S826_DRIVER_BUILD
// These are needed for the Linux demo.
// s826example does not include platform.h
// Please do not remove.
#ifndef RT_SUPPORT
#include <linux/types.h>
#include <stdint.h>
#endif

typedef unsigned int uint;
typedef uint64_t uint64;
#endif
#endif
#endif

//////////////////////////// SYSTEM /////////////////////////////////////

S826_API int S826_CC S826_VersionRead               (uint board, uint *api, uint *driver, uint *bdrev, uint *fpgarev);

#ifndef RT_SUPPORT
S826_API int S826_CC S826_SystemOpen                (void);
#else
S826_API int S826_CC S826_SystemOpen                (int index, uint *idx);
#endif


S826_API int S826_CC S826_SystemClose               (void);
S826_API int S826_CC S826_TimestampRead             (uint board, uint *timestamp);

//////////////////////////// SAFEMODE ///////////////////////////////////

S826_API int S826_CC S826_SafeControlRead           (uint board, uint *settings);
S826_API int S826_CC S826_SafeControlWrite          (uint board, uint settings, uint mode);
S826_API int S826_CC S826_SafeWrenRead              (uint board, uint *enable);
S826_API int S826_CC S826_SafeWrenWrite             (uint board, uint enable);

/////////////////////////////// ADC /////////////////////////////////////

S826_API int S826_CC S826_AdcCalRead                (uint board, double slope[4], int offset[4], uint *valid);
S826_API int S826_CC S826_AdcCalWrite               (uint board, const double slope[4], const int offset[4]);
S826_API int S826_CC S826_AdcEnableRead             (uint board, uint *enable);
S826_API int S826_CC S826_AdcEnableWrite            (uint board, uint enable);
S826_API int S826_CC S826_AdcRawRead                (uint board, int buf[32], uint *timestamps, uint *slotlist, uint tmax);
S826_API int S826_CC S826_AdcRead                   (uint board, int buf[16], uint *timestamps, uint *slotlist, uint tmax);
S826_API int S826_CC S826_AdcSlotConfigRead         (uint board, uint slot, uint *chan, uint *tsettle, uint *range);
S826_API int S826_CC S826_AdcSlotConfigWrite        (uint board, uint slot, uint chan, uint tsettle, uint range);
S826_API int S826_CC S826_AdcSlotlistRead           (uint board, uint *slotlist);
S826_API int S826_CC S826_AdcSlotlistWrite          (uint board, uint slotlist, uint mode);
S826_API int S826_CC S826_AdcStatusRead             (uint board, uint *slotlist);
S826_API int S826_CC S826_AdcTrigModeRead           (uint board, uint *trigmode);
S826_API int S826_CC S826_AdcTrigModeWrite          (uint board, uint trigmode);

#ifndef RT_SUPPORT
S826_API int S826_CC S826_AdcWaitCancel             (uint board, uint slotlist);
#endif
///////////////////////////////// DAC ////////////////////////////////////

S826_API int S826_CC S826_DacCalRead                (uint board, double scalars[4], uint *valid);
S826_API int S826_CC S826_DacCalWrite               (uint board, const double scalars[4]);
S826_API int S826_CC S826_DacDataWrite              (uint board, uint chan, uint setpoint, uint safemode);
S826_API int S826_CC S826_DacRangeWrite             (uint board, uint chan, uint range, uint safemode);
S826_API int S826_CC S826_DacRawRead                (uint board, uint chan, uint *setpoint);
S826_API int S826_CC S826_DacRawWrite               (uint board, uint chan, uint setpoint);
S826_API int S826_CC S826_DacRead                   (uint board, uint chan, uint *range, uint *setpoint, uint safemode);

/////////////////////////// COUNTERS //////////////////////////////

S826_API int S826_CC S826_CounterCompareRead        (uint board, uint chan, uint regid, uint *counts);
S826_API int S826_CC S826_CounterCompareWrite       (uint board, uint chan, uint regid, uint counts);
S826_API int S826_CC S826_CounterExtInRoutingRead   (uint board, uint chan, uint *route);
S826_API int S826_CC S826_CounterExtInRoutingWrite  (uint board, uint chan, uint route);
S826_API int S826_CC S826_CounterFilterRead         (uint board, uint chan, uint *cfg);
S826_API int S826_CC S826_CounterFilterWrite        (uint board, uint chan, uint cfg);
S826_API int S826_CC S826_CounterModeRead           (uint board, uint chan, uint *modeinfo);
S826_API int S826_CC S826_CounterModeWrite          (uint board, uint chan, uint mode);
S826_API int S826_CC S826_CounterPreload            (uint board, uint chan, uint level, uint sticky);
S826_API int S826_CC S826_CounterPreloadRead        (uint board, uint chan, uint reg, uint *counts);
S826_API int S826_CC S826_CounterPreloadWrite       (uint board, uint chan, uint reg, uint counts);
S826_API int S826_CC S826_CounterRead               (uint board, uint chan, uint *counts);
S826_API int S826_CC S826_CounterSnapshot           (uint board, uint chan);
S826_API int S826_CC S826_CounterSnapshotConfigRead (uint board, uint chan, uint *ctrl);
S826_API int S826_CC S826_CounterSnapshotConfigWrite(uint board, uint chan, uint ctrl, uint mode);
S826_API int S826_CC S826_CounterSnapshotRead       (uint board, uint chan, uint *value, uint *tstamp, uint *reason, uint tmax);
S826_API int S826_CC S826_CounterStateWrite         (uint board, uint chan, uint run);
S826_API int S826_CC S826_CounterStatusRead         (uint board, uint chan, uint *status);


S826_API int S826_CC S826_CounterWaitCancel         (uint board, uint chan);

///////////////////////////// DIO //////////////////////////////////

S826_API int S826_CC S826_DioCapEnablesRead         (uint board, uint rising[2], uint falling[2]);
S826_API int S826_CC S826_DioCapEnablesWrite        (uint board, const uint rising[2], const uint falling[2], uint mode);
S826_API int S826_CC S826_DioCapRead                (uint board, uint chanlist[2], uint waitall, uint tmax);
S826_API int S826_CC S826_DioFilterRead             (uint board, uint *interval, uint enables[2]);
S826_API int S826_CC S826_DioFilterWrite            (uint board, const uint interval, const uint enables[2]);
S826_API int S826_CC S826_DioInputRead              (uint board, uint data[2]);
S826_API int S826_CC S826_DioOutputRead             (uint board, uint data[2]);
S826_API int S826_CC S826_DioOutputWrite            (uint board, const uint data[2], uint mode);
S826_API int S826_CC S826_DioOutputSourceRead       (uint board, uint data[2]);
S826_API int S826_CC S826_DioOutputSourceWrite      (uint board, const uint data[2]);
S826_API int S826_CC S826_DioSafeEnablesRead        (uint board, uint enables[2]);
S826_API int S826_CC S826_DioSafeEnablesWrite       (uint board, const uint enables[2]);
S826_API int S826_CC S826_DioSafeRead               (uint board, uint data[2]);
S826_API int S826_CC S826_DioSafeWrite              (uint board, const uint data[2], uint mode);
#ifndef RT_SUPPORT
S826_API int S826_CC S826_DioWaitCancel             (uint board, const uint data[2]);
#endif

/////////////////////////// VARIABLES (BURIED DIO) //////////////////////////

S826_API int S826_CC S826_VirtualRead               (uint board, uint *data);
S826_API int S826_CC S826_VirtualWrite              (uint board, const uint data, uint mode);
S826_API int S826_CC S826_VirtualSafeRead           (uint board, uint *data);
S826_API int S826_CC S826_VirtualSafeWrite          (uint board, const uint data, uint mode);
S826_API int S826_CC S826_VirtualSafeEnablesRead    (uint board, uint *enables);
S826_API int S826_CC S826_VirtualSafeEnablesWrite   (uint board, const uint enables);

///////////////////////////// WATCHDOG //////////////////////////////////

S826_API int S826_CC S826_WatchdogEnableRead        (uint board, uint *enable);
S826_API int S826_CC S826_WatchdogEnableWrite       (uint board, uint enable);
S826_API int S826_CC S826_WatchdogConfigRead        (uint board, uint *config, uint timers[5]);
S826_API int S826_CC S826_WatchdogConfigWrite       (uint board, uint config, uint timers[5]);
S826_API int S826_CC S826_WatchdogStatusRead        (uint board, uint *status);
S826_API int S826_CC S826_WatchdogKick              (uint board, uint data);
S826_API int S826_CC S826_WatchdogEventWait         (uint board, uint tmax);
#ifndef RT_SUPPORT
S826_API int S826_CC S826_WatchdogWaitCancel        (uint board);
#endif
/////////// FOR INTERNAL SENSORAY USE ONLY //////////////////////////////////

S826_API int S826_CC S826_RamRead                   (uint board, uint addr, uint *data);
S826_API int S826_CC S826_RamWrite                  (uint board, uint addr, uint data);

S826_API int S826_CC S826_EepromReadByte            (uint board, uint addr, uint *data);
S826_API int S826_CC S826_EepromReadQuadlet         (uint board, uint addr, uint *data);
S826_API int S826_CC S826_EepromWriteByte           (uint board, uint addr, uint data);
S826_API int S826_CC S826_EepromWriteQuadlet        (uint board, uint addr, uint data);

S826_API void S826_CC S826_WriteReg                 (uint board, uint offset, uint val);
S826_API uint S826_CC S826_ReadReg                  (uint board, uint offset);
S826_API int S826_CC S826_WriteBridge               (uint board, uint lcs, uint offset, uint value, uint rindex, uint rcode);
S826_API int S826_CC S826_ReadBridge                (uint board, uint lcs, uint offset, uint *value, uint rindex, uint rcode);


#ifdef __cplusplus
}
#endif

#endif // #ifndef _INC_826API_H_
