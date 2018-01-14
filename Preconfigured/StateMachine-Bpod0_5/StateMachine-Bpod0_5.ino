/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Bpod_Gen2 repository
  Copyright (C) 2017 Sanworks LLC, Stony Brook, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/
// Bpod State Machine Firmware Ver. 20
//
// SYSTEM SETUP:
//
// IF COMPILING FOR Arduino DUE, Requires a small modification to Arduino, for useful PWM control of light intensity with small (1-10ms) pulses:
// change the PWM_FREQUENCY and TC_FREQUENCY constants to 50000 in the file /Arduino/hardware/arduino/sam/variants/arduino_due_x/variant.h
// or in Windows, \Users\Username\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.7\variants\arduino_due_x\variant.h
//
// IF COMPILING FOR TEENSY 3.6, Requires modifications to Teensy core files:
// In the folder /arduino-1.8.X/hardware/teensy/avr/cores/teensy3, modify the following line in each of the 5 files listed below:
// #define SERIAL1_RX_BUFFER_SIZE 64  --> #define SERIAL1_RX_BUFFER_SIZE 256
// IN FILES: serial1.c, serial2.c, serial3.c, serial4.c, serial5.c

//////////////////////////////////////////
// Set hardware series (0.5, 0.7+, etc)  /
//////////////////////////////////////////
// 1 = Bpod 0.5 (Arduino Due); 2 = Bpod 0.7-0.9 (Arduino Due); 3 = Bpod 2.0 (Teensy 3.6)

#define MACHINE_TYPE 1 

//////////////////////////////////////////
//    State Machine Feature Profile      /
//////////////////////////////////////////
// Profile of internal state machine features (max numbers of global timers, counters and conditions).
// Declaring more of these requires more MCU sRAM, often in ways that are non-linear. It is safe to add 
// profiles here to reallocate memory as needed, at the line beginning: #if SM_FEATURE_PROFILE == 0 
// 0 = Bpod Native on HW 0.5-0.9 (5,5,5)
// 1 = Bpod Native on HW 2.0 (16,8,16)
// 2 = Bpod for BControl on HW 0.7-0.9 (8,2,8) 
// 3 = Bpod for Bcontrol on HW 1.0 (20,2,20)

#define SM_FEATURE_PROFILE 0

#if SM_FEATURE_PROFILE == 0
  #define MAX_GLOBAL_TIMERS 5
  #define MAX_GLOBAL_COUNTERS 5
  #define MAX_CONDITIONS 5
#elif SM_FEATURE_PROFILE == 1
  #define MAX_GLOBAL_TIMERS 16
  #define MAX_GLOBAL_COUNTERS 8
  #define MAX_CONDITIONS 16
#elif SM_FEATURE_PROFILE == 2
  #define MAX_GLOBAL_TIMERS 8
  #define MAX_GLOBAL_COUNTERS 2
  #define MAX_CONDITIONS 8
#elif SM_FEATURE_PROFILE == 3
  #define MAX_GLOBAL_TIMERS 20
  #define MAX_GLOBAL_COUNTERS 2
  #define MAX_CONDITIONS 20
#endif

//////////////////////////////////////////
//          Set Firmware Version         /
//////////////////////////////////////////
// Current firmware version (single firmware file, compiles for MachineTypes set above).

#define FIRMWARE_VERSION 20

//////////////////////////////////////////
//      Live Timestamp Transmission      /
//////////////////////////////////////////
// 0 to return timestamps after trial, 1 to return timestamps during trial along with event byte-codes 

#define LIVE_TIMESTAMPS 1

//////////////////////////////////////////
//    Ethernet Communication Option      /
//////////////////////////////////////////
// Defines the state machine's communication channel to the PC. 0 = USB (default), 1 = Ethernet (w/ Bpod Ethernet Module)
// IMPORTANT: PC via Ethernet requires State Machine v2.0 or newer.

#define ETHERNET_COM 0

//////////////////////////////////////////
//          Board configuration          /
//////////////////////////////////////////

// Board configuration (universal)
#define STATUS_LED_ENABLED 1 // Set to 0 to disable the board's red/green/blue status LED

// Board configuration (machine-specific)
#if MACHINE_TYPE < 3
  #include "DueTimer.h"
#endif
#include <SPI.h>
#include "ArCOM.h" // ArCOM is a serial interface wrapper developed by Sanworks, 
                   // to streamline transmission of datatypes and arrays over USB and UART
#if ETHERNET_COM == 1
  #include "ArCOMvE.h" // Special variant of ArCOM specialized for transmission via Bpod Ethernet module
#endif

#if MACHINE_TYPE == 1
  ArCOM PC(SerialUSB); // Creates an ArCOM object called PC, wrapping SerialUSB (or in some cases, a different serial channel with an Ethernet link to the PC)
  ArCOM Module1(Serial1); // Creates an ArCOM object called Module1, wrapping Serial1
  ArCOM Module2(Serial2); 
#elif MACHINE_TYPE == 2
  ArCOM PC(SerialUSB); 
  ArCOM Module1(Serial1); 
  ArCOM Module2(Serial2);
  ArCOM Module3(Serial3);
#elif MACHINE_TYPE == 3
  #if ETHERNET_COM == 0 
    ArCOM PC(SerialUSB); 
  #else
    ArCOMvE PC(Serial5); // Creates an ArCOM object called PC, wrapping Serial (for Teensy 3.6)
  #endif
  ArCOM Module1(Serial1); // Creates an ArCOM object called Module1, wrapping Serial1
  ArCOM Module2(Serial3); 
  ArCOM Module3(Serial2); 
  ArCOM Module4(Serial4); 
  #if ETHERNET_COM == 0 
    ArCOM Module5(Serial5); 
  #endif
#endif

////////////////////////////////////////
// State machine hardware description  /
////////////////////////////////////////
// Two pairs of arrays describe the hardware as it appears to the state machine: inputHW / inputCh, and outputHW / outputCh.
// In these arrays, the first row codes for hardware type. U = UART, X = USB, S = SPI,  D = digital, B = BNC (digital/inverted), W = Wire (digital/inverted), V = Valve (digital) 
// P = port (digital channel if input, PWM channel if output). Channels must be listed IN THIS ORDER (this constraint allows efficient code at runtime). 
// The digital,BNC or wire channel currently replaced by 'Y' is the sync channel (none by default).
// The second row lists the physical input and output channels on Arduino for B,W,P, and the SPI CS pin is listed for S. Use 0 for UART and USB.

#if MACHINE_TYPE == 1 // Bpod state machine v0.5
    byte InputHW[] = {'U','U','X','B','B','W','W','W','W','P','P','P','P','P','P','P','P'};
    byte InputCh[] = {0,0,0,11,10,35,33,31,29,28,30,32,34,36,38,40,42};                                         
    byte OutputHW[] = {'U','U','X','B','B','W','W','W','W','P','P','P','P','P','P','P','P','V','V','V','V','V','V','V','V'};
    byte OutputCh[] = {0,0,0,25,24,43,41,39,37,9,8,7,6,5,4,3,2,22,22,22,22,22,22,22,22};   
#elif MACHINE_TYPE == 2 // Bpod State Machine r0.7+
    byte InputHW[] = {'U','U','U','X','B','B','W','W','P','P','P','P','P','P','P','P'};
    byte InputCh[] = {0,0,0,0,10,11,31,29,28,30,32,34,36,38,40,42};                                         
    byte OutputHW[] = {'U','U','U','X','B','B','W','W','W','P','P','P','P','P','P','P','P','V','V','V','V','V','V','V','V'};
    byte OutputCh[] = {0,0,0,0,25,24,43,41,39,9,8,7,6,5,4,3,2,22,22,22,22,22,22,22,22};  
#elif MACHINE_TYPE == 3 // Bpod State Machine r2.0+
  #if ETHERNET_COM == 0
    byte InputHW[] = {'U','U','U','U','U','X','B','B','P','P','P','P'};
    byte InputCh[] = {0,0,0,0,0,0,6,5,39,38,18,15};                                         
    byte OutputHW[] = {'U','U','U','U','U','X','B','B','P','P','P','P','V','V','V','V'};
    byte OutputCh[] = {0,0,0,0,0,0,4,3,37,14,16,17,23,22,20,21};  
  #else
    byte InputHW[] = {'U','U','U','U','X','B','B','P','P','P','P'};
    byte InputCh[] = {0,0,0,0,0,6,5,39,38,18,15};                                         
    byte OutputHW[] = {'U','U','U','U','X','B','B','P','P','P','P','V','V','V','V'};
    byte OutputCh[] = {0,0,0,0,0,4,3,37,14,16,17,23,22,20,21}; 
  #endif
#endif

// State machine meta information
const byte nInputs = sizeof(InputHW);
const byte nOutputs = sizeof(OutputHW);
#if MACHINE_TYPE == 1 // State machine (Bpod 0.5)
  const byte nSerialChannels = 3; // Must match total of 'U' and 'X' in InputHW (above)
  const byte maxSerialEvents = 30; // Must be a multiple of nSerialChannels
  const int MaxStates = 128;
  const int SerialBaudRate = 115200;
#elif MACHINE_TYPE == 2 // Bpod State Machine r0.7+
  const byte nSerialChannels = 4; 
  const byte maxSerialEvents = 60;
  const int MaxStates = 256;
  const int SerialBaudRate = 1312500;
#elif MACHINE_TYPE == 3  // Teensy 3.6 based state machines (r2.0+)
  #if ETHERNET_COM == 0
    const byte nSerialChannels = 6; 
    const byte maxSerialEvents = 90;
  #else
    const byte nSerialChannels = 5; 
    const byte maxSerialEvents = 75;
  #endif
  const int MaxStates = 256;
  const int SerialBaudRate = 1312500;
#endif

uint16_t timerPeriod = 100; // Hardware timer period, in microseconds (state machine refresh period)

#if MAX_GLOBAL_TIMERS > 16
  #define GLOBALTIMER_TRIG_BYTEWIDTH 4
#elif MAX_GLOBAL_TIMERS > 8
  #define GLOBALTIMER_TRIG_BYTEWIDTH 2
#else
  #define GLOBALTIMER_TRIG_BYTEWIDTH 1
#endif

// Vars to hold number of timers, counters and conditions actually used in the current state matrix
byte nGlobalTimersUsed = MAX_GLOBAL_TIMERS;
byte nGlobalCountersUsed = MAX_GLOBAL_COUNTERS;
byte nConditionsUsed = MAX_CONDITIONS;
                         
// Other hardware pin mapping
#if MACHINE_TYPE == 1
  byte GreenLEDPin = 14;
  byte RedLEDPin = 13;
  byte BlueLEDPin = 12;
  byte valveCSChannel = 22;
#elif MACHINE_TYPE == 2
  byte GreenLEDPin = 33;
  byte RedLEDPin = 13;
  byte BlueLEDPin = 12;
  byte valveCSChannel = 22;
#elif MACHINE_TYPE == 3
  byte GreenLEDPin = 2;
  byte RedLEDPin = 36;
  byte BlueLEDPin = 35;
  byte ValveEnablePin = 19;
  byte valveCSChannel = 0;
#endif

byte fRAMcs = 1;
byte fRAMhold = A3;
byte SyncRegisterLatch = 23;

// Settings for version-specific hardware (initialized in setup)
boolean usesFRAM = false;
boolean usesSPISync = false;
boolean usesSPIValves = false;
boolean usesUARTInputs = false;
boolean isolatorHigh = 0;
boolean isolatorLow = 0;

// Bookmarks (positions of channel types in hardware description vectors, to be calculated in setup)
byte USBInputPos = 0;
byte BNCInputPos = 0;
byte WireInputPos = 0;
byte PortInputPos = 0;
byte USBOutputPos = 0;
byte SPIOutputPos = 0;
byte BNCOutputPos = 0;
byte WireOutputPos = 0;
byte PortOutputPos = 0;
byte ValvePos = 0;

// Parameters

const unsigned long ModuleIDTimeout = 100; // timeout for modules to respond to byte 255 (units = 100us cycles). Byte 255 polls for module hardware information

// Initialize system state vars: 
byte outputState[nOutputs] = {0}; // State of outputs
byte inputState[nInputs+MAX_GLOBAL_TIMERS] = {0}; // Current state of inputs
byte lastInputState[nInputs+MAX_GLOBAL_TIMERS] = {0}; // State of inputs on previous cycle
byte inputOverrideState[nInputs] = {0}; // Set to 1 if user created a virtual event, to prevent hardware reads until user returns low
byte outputOverrideState[nOutputs] = {0}; // Set to 1 when overriding a digital output line. This prevents state changes from affecting the line until it is manually reset.
byte inputEnabled[nInputs] = {0}; // 0 if input disabled, 1 if enabled
byte logicHigh[nInputs] = {0}; // 1 if standard, 0 if inverted input (calculated in setup for each channel, depending on its type)
byte logicLow[nInputs] = {0}; // Inverse of logicHigh (to save computation time)
const byte nDigitalInputs = nInputs - nSerialChannels; // Number of digital input channels
boolean MatrixFinished = false; // Has the system exited the matrix (final state)?
boolean MeaningfulStateTimer = false; // Does this state's timer get us to another state when it expires?
int CurrentState = 1; // What state is the state machine currently in? (State 0 is the final state)
int NewState = 1; // State the system determined it needs to enter next. If different from current state, transition logic proceeds.
int CurrentStateTEMP = 1; // Temporarily holds current state during transition
// Event vars
const byte maxCurrentEvents = 10; // Max number of events that can be recorded during a single 100 microsecond cycle
byte CurrentEvent[maxCurrentEvents] = {0}; // What event code just happened and needs to be handled. Up to 10 can be acquired per 100us loop.
byte CurrentEventBuffer[maxCurrentEvents+6] = {0}; // Current events as packaged for rapid vector transmission 
byte nCurrentEvents = 0; // Index of current event
byte SoftEvent = 0; // What soft event code just happened
// Trial / Event Sync Vars
byte SyncChannel = 255; // 255 if no sync codes, <255 to specify a channel to use for sync
boolean syncOn = false; // If true, sync codes are sent on sync channel
byte SyncChannelHW = 0; // Stores physical pin for sync channel
byte NewSyncChannel = 0; // New digital output channel to use for Sync. (old channel gets reset to its original hardware type)
byte SyncMode = 0; // 0 if low > high on trial start and high < low on trial end, 1 if line toggles with each state change
byte SyncState = 0; // State of the sync line (0 = low, 1 = high)
// Others
boolean smaTransmissionConfirmed = false; // Set to true when the last state machine was successfully received, set to false when starting a transmission
boolean newSMATransmissionStarted = false; // Set to true when beginning a state machine transmission
boolean UARTrelayMode[nSerialChannels] = {false};
byte nModuleEvents[nSerialChannels] = {10}; // Stores number of behavior events assigned to each serial module (10 by default)
uint16_t stateMatrixNBytes = 0; // Number of bytes in the state matrix about to be transmitted
boolean using255BackSignal = 0; // If enabled, only 254 states can be used and going to "state 255" returns system to the previous state


//////////////////////////////////
// Initialize general use vars:  /
//////////////////////////////////
const byte discoveryByte = 222; // Unique byte to send to any serial port on first connection (for USB serial port auto-discovery)
const byte discoveryByteInterval = 100; // ms between discovery byte transmissions
boolean RunStateMatrixASAP = false; // On state matrix transmission, set to 1 if the new state matrix should auto-run after the trial ends
byte CommandByte = 0;  // Op code to specify handling of an incoming USB serial message
byte VirtualEventTarget = 0; // Op code to specify which virtual event type (Port, BNC, etc)
byte VirtualEventData = 0; // State of target
byte Byte1 = 0; byte Byte2 = 0; byte Byte3 = 0; byte Byte4 = 0; // Temporary storage of values read from serial port
byte nPossibleEvents = 0; // possible events in the state machine (computed in setup)
int nStates = 0; // Total number of states in the current state machine
int nEvents = 0; // Total number of events recorded while running current state machine
byte previousState = 0; // Previous state visited. Used if 255 is interpreted as a "back" signal (see using255BackSignal above)
int LEDBrightnessAdjustInterval = 5;
byte LEDBrightnessAdjustDirection = 1;
byte LEDBrightness = 0;
byte serialByteBuffer[4] = {0}; // Stores 1-3 byte messages transmitted as output actions of states

// Vars for state machine definitions. Each matrix relates each state to some inputs or outputs.
const byte InputMatrixSize = maxSerialEvents + nDigitalInputs*2;
byte InputStateMatrix[MaxStates+1][InputMatrixSize] = {0}; // Matrix containing all of Bpod's digital inputs and corresponding state transitions
byte StateTimerMatrix[MaxStates+1] = {0}; // Matrix containing states to move to if the state timer elapses
const byte OutputMatrixSize = nOutputs;
byte OutputStateMatrix[MaxStates+1][OutputMatrixSize] = {0}; // Hardware states for outputs. Serial channels > Digital outputs > Virtual (global timer trigger, global timer cancel, global counter reset)
byte smGlobalCounterReset[MaxStates+1] = {0}; // For each state, global counter to reset.
byte GlobalTimerStartMatrix[MaxStates+1][MAX_GLOBAL_TIMERS] = {0}; // Matrix contatining state transitions for global timer onset events
byte GlobalTimerEndMatrix[MaxStates+1][MAX_GLOBAL_TIMERS] = {0}; // Matrix contatining state transitions for global timer elapse events
byte GlobalCounterMatrix[MaxStates+1][MAX_GLOBAL_COUNTERS] = {0}; // Matrix contatining state transitions for global counter threshold events
byte ConditionMatrix[MaxStates+1][MAX_CONDITIONS] = {0}; // Matrix contatining state transitions for conditions
boolean GlobalTimersTriggered[MAX_GLOBAL_TIMERS] = {0}; // 0 if timer x was not yet triggered, 1 if it was triggered and had not elapsed.
boolean GlobalTimersActive[MAX_GLOBAL_TIMERS] = {0}; // 0 if timer x is inactive (e.g. not triggered, or during onset delay after trigger), 1 if it's active.
byte SerialMessageMatrix[MaxStates+1][nSerialChannels][3]; // Stores a 3-byte serial message for each message byte on each port
byte SerialMessage_nBytes[MaxStates+1][nSerialChannels] = {1}; // Stores the length of each serial message
boolean ModuleConnected[nSerialChannels] = {false}; // true for each channel if a module is connected, false otherwise
byte SyncChannelOriginalType = 0; // Stores sync channel's original hardware type
uint32_t PWMChannel[nOutputs] = {0}; // Stores ARM PWM channel for each PWM output pin (assigned in advance in setup, to speed PWM writes)

// Global timer triggers (data type dependent on number of global timers; using fewer = faster SM switching, extra memory for other configs)
#if GLOBALTIMER_TRIG_BYTEWIDTH == 1
  uint8_t GlobalTimerOnsetTriggers[MAX_GLOBAL_TIMERS] = {0}; // Bits indicate other global timers to trigger when timer turns on (after delay)
  uint8_t smGlobalTimerTrig[MaxStates+1] = {0}; // For each state, global timers to trigger. Bits indicate timers.
  uint8_t smGlobalTimerCancel[MaxStates+1] = {0}; // For each state, global timers to cancel. Bits indicate timers.
#elif GLOBALTIMER_TRIG_BYTEWIDTH == 2
  uint16_t GlobalTimerOnsetTriggers[MAX_GLOBAL_TIMERS] = {0};
  uint16_t smGlobalTimerTrig[MaxStates+1] = {0};
  uint16_t smGlobalTimerCancel[MaxStates+1] = {0};
#elif GLOBALTIMER_TRIG_BYTEWIDTH == 4
  uint32_t GlobalTimerOnsetTriggers[MAX_GLOBAL_TIMERS] = {0};
  uint32_t smGlobalTimerTrig[MaxStates+1] = {0};
  uint32_t smGlobalTimerCancel[MaxStates+1] = {0};
#endif

// Positions of input matrix parts
byte GlobalTimerStartPos = InputMatrixSize; // First global timer event code
byte GlobalTimerEndPos = GlobalTimerStartPos + MAX_GLOBAL_TIMERS;
byte GlobalCounterPos = GlobalTimerEndPos + MAX_GLOBAL_TIMERS; // First global counter event code
byte ConditionPos = GlobalCounterPos + MAX_GLOBAL_COUNTERS; // First condition event code
byte TupPos = ConditionPos+MAX_CONDITIONS; // First Jump event code
byte DigitalInputPos = maxSerialEvents;


byte GlobalTimerChannel[MAX_GLOBAL_TIMERS] = {254}; // Channel code for global timer onset/offset.
byte GlobalTimerOnMessage[MAX_GLOBAL_TIMERS] = {254}; // Message to send when global timer is active (if channel is serial).
byte GlobalTimerOffMessage[MAX_GLOBAL_TIMERS] = {254}; // Message to send when global timer elapses (if channel is serial).
unsigned long GlobalTimerStart[MAX_GLOBAL_TIMERS] = {0}; // Future Times when active global timers will start measuring time
unsigned long GlobalTimerEnd[MAX_GLOBAL_TIMERS] = {0}; // Future Times when active global timers will elapse
unsigned long GlobalTimers[MAX_GLOBAL_TIMERS] = {0}; // Timers independent of states
unsigned long GlobalTimerOnsetDelays[MAX_GLOBAL_TIMERS] = {0}; // Onset delay following global timer trigger
unsigned long GlobalTimerLoopIntervals[MAX_GLOBAL_TIMERS] = {0}; // Configurable delay between global timer loop iterations
byte GlobalTimerLoop[MAX_GLOBAL_TIMERS] = {0}; // Number of loop iterations. 0 = no loop. 1 = loop until shut-off. 2-255 = number of loops to execute
byte GlobalTimerLoopCount[MAX_GLOBAL_TIMERS] = {0}; // When GlobalTimerLoop > 1, counts the number of loops elapsed
boolean GTUsingLoopCounter[MAX_GLOBAL_TIMERS] = {false}; // If this timer's GlobalTimerLoop > 1 (i.e. terminated by loop counter)
byte SendGlobalTimerEvents[MAX_GLOBAL_TIMERS] = {0}; // true if events are returned to the state machine (especially useful to disable in loop mode)
unsigned long GlobalCounterCounts[MAX_GLOBAL_COUNTERS] = {0}; // Event counters
byte GlobalCounterAttachedEvents[MAX_GLOBAL_COUNTERS] = {254}; // Event each event counter is attached to
unsigned long GlobalCounterThresholds[MAX_GLOBAL_COUNTERS] = {0}; // Event counter thresholds (trigger events if crossed)
boolean GlobalCounterHandled[MAX_GLOBAL_COUNTERS] = {false};
byte ConditionChannels[MAX_CONDITIONS] = {254}; // Event each channel a condition is attached to
byte ConditionValues[MAX_CONDITIONS] = {0}; // The value of each condition
const int MaxTimestamps = 10000;
#if MACHINE_TYPE != 2
  unsigned long Timestamps[MaxTimestamps] = {0};
#endif
unsigned long StateTimers[MaxStates+1] = {0}; // Timers for each state
unsigned long StartTime = 0; // System Start Time
uint64_t MatrixStartTimeMicros = 0; // Start time of state matrix (in us)
uint64_t MatrixEndTimeMicros = 0; // End time of state matrix (in us)
uint32_t currentTimeMicros = 0; // Current time (for detecting 32-bit clock rollovers)
uint32_t lastTimeMicros = 0; // Last time read (for detecting  32-bit clock rollovers)
unsigned long nCyclesCompleted= 0; // Number of HW timer cycles since state matrix started
unsigned long StateStartTime = 0; // Session Start Time
unsigned long NextLEDBrightnessAdjustTime = 0; // Used to fade blue light when disconnected
unsigned long CurrentTime = 0; // Current time (units = timer cycles since start; used to control state transitions)
unsigned long TimeFromStart = 0;
uint64_t sessionStartTimeMicros = 0;
uint32_t nMicrosRollovers = 0; // Number of micros() clock rollovers since session start
boolean cycleMonitoring = 0; // if 1, measures time between hardware timer callbacks when state transitions occur
boolean getModuleInfo = false; // If retrieving module info
unsigned long nBytes = 0; // Number of bytes to read (when transmitting module info)
unsigned long CallbackStartTime = 0; // For self-monitoring to detect hardware timer overruns
unsigned long DiscoveryByteTime = 0; // Last time a discovery byte was sent (for USB serial port auto-discovery)
unsigned long nCycles = 0; // Number of cycles measured since event X
byte connectionState = 0; // 1 if connected to MATLAB
byte RunningStateMatrix = 0; // 1  if state matrix is running
byte firstLoop= 0; // 1 if first timer callback in state matrix
int Ev = 0; // Index of current event
byte overrideChan = 0; // Output channel being manually overridden
byte overrideChanState = 0; // State of channel being manually overridden
byte nOverrides = 0; // Number of overrides on a line of the state matrix (for compressed transmission scheme)
byte col = 0; byte val = 0; // col and val are used in state matrix compression scheme
const uint16_t StateMatrixBufferSize = 50000;
#if MACHINE_TYPE > 1
  byte StateMatrixBuffer[StateMatrixBufferSize] = {0}; // Stores next trial's state matrix
#endif
const uint16_t SerialRelayBufferSize = 256;
byte SerialRelayBuffer[SerialRelayBufferSize] = {0}; // Stores bytes to be transmitted to a serial device (i.e. module, USB)
uint16_t bufferPos = 0;
boolean smaPending = false; // If a state matrix is ready to read into the serial buffer (from USB)
boolean smaReady2Load = false; // If a state matrix was read into the serial buffer and is ready to read into sma vars with LoadStateMatrix()
boolean runFlag = false; // True if a command to run a state matrix arrives while an SM transmission is ongoing. Set to false once new SM starts.
uint16_t nSMBytesRead = 0; // For state matrix transmission during trial, where bytes transmitted must be tracked across timer interrupts

union {
  byte Bytes[maxCurrentEvents*4];
  uint32_t Uint32[maxCurrentEvents];
} CurrentTimeStamps; // For trial timestamp conversion

union {
    byte byteArray[4];
    uint16_t uint16;
    uint32_t uint32;
} typeBuffer; // For general purpose type conversion

union {
    byte byteArray[16];
    uint32_t uint32[4];
    uint64_t uint64[2];
} timeBuffer; // For time transmission on trial end

#if MACHINE_TYPE == 3
  IntervalTimer hardwareTimer;
#endif


void setup() {
// Resolve hardware peripherals from machine type
if (MACHINE_TYPE == 1) {
    usesFRAM = false;
    usesSPISync = true;
    usesSPIValves = true;
    usesUARTInputs = false;
    isolatorHigh = 1;
    isolatorLow = 0;
} else if (MACHINE_TYPE == 2) {
    usesFRAM = true;
    usesSPISync = false;
    usesSPIValves = true;
    usesUARTInputs = true;
    isolatorHigh = 0;
    isolatorLow = 1;
} else if (MACHINE_TYPE == 3) {
    usesFRAM = false;
    usesSPISync = false;
    usesSPIValves = false;
    usesUARTInputs = true;
    isolatorHigh = 0;
    isolatorLow = 1;
}
// Find Bookmarks (positions of channel types in hardware description vectors)
for (int i = 0; i < nInputs; i++) {
  if ((InputHW[i] == 'X') && (USBInputPos == 0)) {
    USBInputPos = i;
  }
  if ((InputHW[i] == 'B') && (BNCInputPos == 0)) {
    BNCInputPos = i;
  }
  if ((InputHW[i] == 'W') && (WireInputPos == 0)) {
    WireInputPos = i;
  }
  if ((InputHW[i] == 'P') && (PortInputPos == 0)) {
    PortInputPos = i;
  }
}
for (int i = 0; i < nOutputs; i++) {
  if ((OutputHW[i] == 'X') && (USBOutputPos == 0)) {
    USBOutputPos = i;
  }
  if ((OutputHW[i] == 'B') && (BNCOutputPos == 0)) {
    BNCOutputPos = i;
  }
  if ((OutputHW[i] == 'W') && (WireOutputPos == 0)) {
    WireOutputPos = i;
  }
  if ((OutputHW[i] == 'P') && (PortOutputPos == 0)) {
    PortOutputPos = i;
  }
  if ((OutputHW[i] == 'V') && (ValvePos == 0)) {
    ValvePos = i;
  }
}  
  // Configure input channels
  Byte1 = 0;
  for (int i = 0; i < nInputs; i++) {
    switch (InputHW[i]) {
      case 'D':
        inputState[i] = 0;
        lastInputState[i] = 0;
        inputEnabled[i] = 1;
        logicHigh[i] = 1;
        logicLow[i] = 0;
      break;
      case 'B':
      case 'W':
        pinMode(InputCh[i], INPUT);
        inputEnabled[i] = 1;
        #if MACHINE_TYPE > 1
          inputState[i] = 1;
          lastInputState[i] = 1;
        #endif
        logicHigh[i] = isolatorHigh;
        logicLow[i] = isolatorLow;
      break; 
      case 'P':
        pinMode(InputCh[i], INPUT_PULLUP);
        logicHigh[i] = 1;
      break;  
      case 'U': 
          switch(Byte1) {
            case 0:
              Serial1.begin(SerialBaudRate); Byte1++;
            break;
            case 1:
              Serial2.begin(SerialBaudRate); Byte1++;
            break;
            #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
                case 2:
                  Serial3.begin(SerialBaudRate); Byte1++;
                break;
            #endif  
            #if MACHINE_TYPE == 3
              case 3:
                Serial4.begin(SerialBaudRate); Byte1++;
              break;
              #if ETHERNET_COM == 0
                case 4:
                  Serial5.begin(SerialBaudRate); Byte1++;
                break;
              #endif
            #endif
          }
      break;
    }
  }
  resetSerialMessages();
  Byte1 = 0;
  // Configure digital output channels
  for (int i = 0; i < nOutputs; i++) {
    switch (OutputHW[i]) {
      case 'D':
      case 'B':
      case 'W':
      case 'V':
        pinMode(OutputCh[i], OUTPUT);
        digitalWrite(OutputCh[i], LOW);
      break;
      case 'P':
        pinMode(OutputCh[i], OUTPUT);
        analogWrite(OutputCh[i], 0);
        #if MACHINE_TYPE < 3
          PWMChannel[i] = g_APinDescription[OutputCh[i]].ulPWMChannel;
        #endif
      break;
    }
  }
  // Start UART --> Ethernet
  #if ETHERNET_COM == 1 && MACHINE_TYPE == 3
     Serial5.begin(2625000);
  #endif  
  Byte1 = 0;
  pinMode(RedLEDPin, OUTPUT);
  pinMode(GreenLEDPin, OUTPUT);
  pinMode(BlueLEDPin, OUTPUT);
  #if MACHINE_TYPE == 3
    pinMode(ValveEnablePin, OUTPUT);
    digitalWrite(ValveEnablePin, LOW);
  #endif
  if (usesFRAM) {
    pinMode(fRAMcs, OUTPUT); // CS pin for the fRAM IC
    pinMode(fRAMhold, OUTPUT); // Hold pin for the fRAM IC
    digitalWrite(fRAMcs, HIGH);
    digitalWrite(fRAMhold, HIGH);
  }
  if (usesSPISync) {
     pinMode(SyncRegisterLatch, OUTPUT); // CS pin for sync shift register IC
     digitalWrite(SyncRegisterLatch, LOW);
  }
  CurrentEventBuffer[0] = 1;
  SPI.begin();
  updateStatusLED(0); // Begin the blue light display ("Disconnected" state)
  #if MACHINE_TYPE < 3
    Timer3.attachInterrupt(handler); // Timer3 is Arduino Due's hardware timer, which will trigger the function "handler" precisely every (timerPeriod) us
    Timer3.start(timerPeriod); // Start HW timer
  #else
    hardwareTimer.begin(handler, timerPeriod); // hardwareTimer is Teensy 3.6's hardware timer
  #endif
}

void loop() {
  if (!RunningStateMatrix) {
    relayModuleBytes();
  }
  #if MACHINE_TYPE > 1
    if (smaPending) { // If a request to read a new state matrix arrived during a trial, read here at lower priority (in free time between timer interrupts)
      PC.readByteArray(StateMatrixBuffer, stateMatrixNBytes);
      smaTransmissionConfirmed = true;
      smaReady2Load = true;
      smaPending = false;
    }
  #endif
  currentTimeMicros = micros();
  if (currentTimeMicros < lastTimeMicros) {
    nMicrosRollovers++;
  }
  lastTimeMicros = currentTimeMicros;
}

void handler() { // This is the timer handler function, which is called every (timerPeriod) us
  if (connectionState == 0) { // If not connected to Bpod software
    if (millis() - DiscoveryByteTime > discoveryByteInterval) { // At a fixed interval, send discovery byte to any connected USB serial port
      #if ETHERNET_COM == 0
        PC.writeByte(discoveryByte);
      #endif
      DiscoveryByteTime = millis();
    }
    updateStatusLED(1); // Update the indicator LED (cycles blue intensity)
  }
  if (runFlag) { // If an SM run command was delayed because an SM transmission is ongoing
    if (smaTransmissionConfirmed) {
      runFlag = false;
      startSM();
    }
  }
  if (getModuleInfo) { // If a request was sent for connected modules to return self description (request = byte 255)
    nCycles++; // Count cycles since request was sent
    if (nCycles > ModuleIDTimeout) { // If modules have had time to reply
      getModuleInfo = false; 
      relayModuleInfo(Module1, 1); // Function transmits 0 if no module replied, 1 if found, followed by length of description(bytes), then description
      relayModuleInfo(Module2, 2);
      #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
        relayModuleInfo(Module3, 3);
      #endif
      #if MACHINE_TYPE == 3
        relayModuleInfo(Module4, 4);
        #if ETHERNET_COM == 0
          relayModuleInfo(Module5, 5);
        #endif
      #endif
    }
  }
  if ((PC.available() > 0) && (smaPending == false)) { // If a message has arrived on the USB serial port
    CommandByte = PC.readByte();  // P for Program, R for Run, O for Override, 6 for Handshake, F for firmware version, etc
    switch (CommandByte) {
      case '6':  // Initialization handshake
        connectionState = 1;
        updateStatusLED(2);
        PC.writeByte(53);
        delayMicroseconds(100000);
        PC.flush();
        resetSessionClock();
        resetSerialMessages();
        disableModuleRelays();
        #if MACHINE_TYPE == 3
          digitalWrite(ValveEnablePin, HIGH); // Enable valve driver
        #endif
      break;
      case 'F':  // Return firmware and machine type
        PC.writeUint16(FIRMWARE_VERSION);
        PC.writeUint16(MACHINE_TYPE);
      break;
      case '*': // Reset session clock
        resetSessionClock();
        PC.writeByte(1);
      break;
      case '$': // Pause ongoing trial (We recommend using computer-side pauses between trials, to keep data uniform)
        RunningStateMatrix = PC.readByte();
      break;
      case 'G':  // Return timestamp transmission scheme
        PC.writeByte(LIVE_TIMESTAMPS);
      break;
      case 'H': // Return local hardware configuration
        PC.writeUint16(MaxStates);
        PC.writeUint16(timerPeriod);
        PC.writeByte(maxSerialEvents);
        PC.writeByte(MAX_GLOBAL_TIMERS);
        PC.writeByte(MAX_GLOBAL_COUNTERS);
        PC.writeByte(MAX_CONDITIONS);
        PC.writeByte(nInputs);
        PC.writeByteArray(InputHW, nInputs);
        PC.writeByte(nOutputs);
        for (int i = 0; i < nOutputs; i++) {
          if (OutputHW[i] == 'Y') {
             PC.writeByte(SyncChannelOriginalType);
          } else {
             PC.writeByte(OutputHW[i]);
          }
        }
      break;
      case 'M': // Probe for connected modules and return module information
        while (Module1.available() > 0 ) {
           Module1.readByte();
        }
        while (Module2.available() > 0 ) {
           Module2.readByte();
        }
        #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
          while (Module3.available() > 0 ) {
            Module3.readByte();
          }
        #endif
        #if MACHINE_TYPE == 3
          while (Module4.available() > 0 ) {
             Module4.readByte();
          }
          #if ETHERNET_COM == 0
            while (Module5.available() > 0 ) {
               Module5.readByte();
            }
          #endif
        #endif
        Module1.writeByte(255);
        Module2.writeByte(255);
        #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
          Module3.writeByte(255);
        #endif
        #if MACHINE_TYPE == 3
          Module4.writeByte(255);
          #if ETHERNET_COM == 0
            Module5.writeByte(255);
          #endif
        #endif
        nCycles = 0; // Number of cycles since request was sent
        getModuleInfo = true; // Outgoing serial messages are not sent until after timer handler completes - so replies are forwarded to USB in the main loop.
      break;
      case '%': // Set number of events per module (this should be run from MATLAB after all modules event # requests are received by MATLAB and a determination is made how to allocate.
        PC.readByteArray(nModuleEvents, nSerialChannels);
        PC.writeByte(1);
      break;
      case 'E': // Enable ports
        for (int i = 0; i < nInputs; i++) {
          inputEnabled[i] = PC.readByte();
        }
        PC.writeByte(1);
      break;
      case 'J': // set serial module relay mode (when not running a state machine, relays one port's incoming bytes to MATLAB/Python
        disableModuleRelays();
        clearSerialBuffers();
        Byte1 = PC.readByte();
        Byte2 = PC.readByte();
        UARTrelayMode[Byte1] = Byte2;
      break;
      case 'K': // Set sync channel and mode
      NewSyncChannel = PC.readByte();
      SyncMode = PC.readByte();
      if (!usesSPISync) {
        if (NewSyncChannel != SyncChannel){ 
          if (NewSyncChannel == 255) {
            if (SyncChannel < nOutputs) {
              OutputHW[SyncChannel] = SyncChannelOriginalType;
            }
            syncOn = false;
          } else {
            if (NewSyncChannel < nOutputs) {
              if (SyncChannel < 255) {
                if (OutputHW[SyncChannel] == 'Y') {
                  OutputHW[SyncChannel] = SyncChannelOriginalType;
                }
              }
              SyncChannelOriginalType = OutputHW[NewSyncChannel];
              OutputHW[NewSyncChannel] = 'Y';
              syncOn = true;
              SyncChannelHW = OutputCh[NewSyncChannel];
            }
          }
          SyncChannel = NewSyncChannel;
        }
      }
      PC.writeByte(1);
      break;
      case 'O':  // Override digital hardware state
        overrideChan = PC.readByte();
        overrideChanState = PC.readByte();
        switch (OutputHW[overrideChan]) {
          case 'D':
          case 'B':
          case 'W':
            digitalWriteDirect(OutputCh[overrideChan], overrideChanState);
            outputOverrideState[overrideChan] = overrideChanState;
          break;
          case 'V':
            if (usesSPIValves) {
              outputState[overrideChan] = overrideChanState;
              valveWrite();
            } else {
              digitalWriteDirect(OutputCh[overrideChan], overrideChanState);
            }
            outputOverrideState[overrideChan] = overrideChanState;
          break;
          case 'P':
            analogWrite(OutputCh[overrideChan], overrideChanState);
            if (overrideChanState > 0) {
              outputOverrideState[overrideChan] = true;
            } else {
              outputOverrideState[overrideChan] = false;
            }
          break;
        }
      break;
      case 'I': // Read and return digital input line states (for debugging)
        Byte1 = PC.readByte();
        Byte2 = digitalReadDirect(InputCh[Byte1]);
        Byte2 = (Byte2 == logicHigh[Byte1]);
        PC.writeByte(Byte2);
      break;
      case 'Z':  // Bpod governing machine has closed the client program
        disableModuleRelays();
        connectionState = 0;
        connectionState = 0;
        PC.writeByte('1');
        updateStatusLED(0);
        DiscoveryByteTime = millis();
        #if MACHINE_TYPE == 3
          digitalWrite(ValveEnablePin, LOW); // Disable valve driver
        #endif
      break;
      case 'S': // Echo Soft code.
        VirtualEventData = PC.readByte();
        PC.writeByte(2);
        PC.writeByte(VirtualEventData);
      break;
      case 'T': // Receive bytes from USB and send to hardware serial channel 1-5
        Byte1 = PC.readByte() - 1; // Serial channel
        nBytes = PC.readUint8();
        switch (Byte1) {
          case 0:
            PC.readByteArray(SerialRelayBuffer, nBytes);
            Module1.writeByteArray(SerialRelayBuffer, nBytes);
          break;
          case 1:
            PC.readByteArray(SerialRelayBuffer, nBytes);
            Module2.writeByteArray(SerialRelayBuffer, nBytes);
          break;
          #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
            case 2:
              PC.readByteArray(SerialRelayBuffer, nBytes);
              Module3.writeByteArray(SerialRelayBuffer, nBytes);
            break;
          #endif
          #if MACHINE_TYPE == 3
            case 3:
              PC.readByteArray(SerialRelayBuffer, nBytes);
              Module4.writeByteArray(SerialRelayBuffer, nBytes);
            break;
            #if ETHERNET_COM == 0
              case 4:
                  PC.readByteArray(SerialRelayBuffer, nBytes);
                  Module5.writeByteArray(SerialRelayBuffer, nBytes);
              break;
            #endif
          #endif
         }
      break;
      case 'U': // Recieve serial message index from USB and send corresponding message to hardware serial channel 1-5
        Byte1 = PC.readByte() - 1;
        Byte2 = PC.readByte();
        Byte3 = SerialMessage_nBytes[Byte2][Byte1];
        for (int i = 0; i < Byte3; i++) {
           serialByteBuffer[i] = SerialMessageMatrix[Byte2][Byte1][i];
        }
        switch (Byte1) {
          case 0:
            Module1.writeByteArray(serialByteBuffer, Byte3);
          break;
          case 1:
            Module2.writeByteArray(serialByteBuffer, Byte3);
          break;
          #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
            case 2:
              Module3.writeByteArray(serialByteBuffer, Byte3);
            break;
          #endif
          #if MACHINE_TYPE == 3
            case 3:
              Module4.writeByteArray(serialByteBuffer, Byte3);
            break;
            #if ETHERNET_COM == 0
              case 4:
                Module5.writeByteArray(serialByteBuffer, Byte3);
              break;
            #endif
          #endif
        }
        break;
      case 'L': // Load serial message library
        Byte1 = PC.readByte(); // Serial Channel
        Byte2 = PC.readByte(); // nMessages arriving
        for (int i = 0; i < Byte2; i++) {
          Byte3 = PC.readByte(); // Message Index
          Byte4 = PC.readByte(); // Message Length
          SerialMessage_nBytes[Byte3][Byte1] = Byte4;
          for (int j = 0; j < Byte4; j++) {
            SerialMessageMatrix[Byte3][Byte1][j] = PC.readByte();
          }
        }
        PC.writeByte(1);
      break;
      case '>': // Reset serial messages to equivalent byte codes (i.e. message# 4 = one byte, 0x4)
        resetSerialMessages();
        PC.writeByte(1);
      break;
      case 'V': // Manual override: execute virtual event
        VirtualEventTarget = PC.readByte();
        VirtualEventData = PC.readByte();
        if (RunningStateMatrix) {
           inputState[VirtualEventTarget] = VirtualEventData;
           inputOverrideState[VirtualEventTarget] = true;
        }
      break;
      case '~': // USB soft code
        SoftEvent = PC.readByte();
        if (!RunningStateMatrix) {
          SoftEvent = 255; // 255 = No Event
        }
      break;
      case 'C': // Get new compressed state matrix from MATLAB/Python 
        newSMATransmissionStarted = true;
        smaTransmissionConfirmed = false;
        PC.readByteArray(typeBuffer.byteArray, 4);
        RunStateMatrixASAP = typeBuffer.byteArray[0];
        using255BackSignal = typeBuffer.byteArray[1];
        typeBuffer.byteArray[0] = typeBuffer.byteArray[2];
        typeBuffer.byteArray[1] = typeBuffer.byteArray[3];
        stateMatrixNBytes = typeBuffer.uint16;
        if (stateMatrixNBytes < StateMatrixBufferSize) {
          if (RunningStateMatrix) {
            #if MACHINE_TYPE > 1
              nSMBytesRead = 0;
              smaPending = true;
            #else
              PC.writeByte(0);
            #endif
          } else {
            #if MACHINE_TYPE > 1
              PC.readByteArray(StateMatrixBuffer, stateMatrixNBytes); // Read data in 1 batch operation (much faster than item-wise)
              smaTransmissionConfirmed = true;
            #endif
            loadStateMatrix(); // Loads the state matrix from the buffer into the relevant variables
            if (RunStateMatrixASAP) {
              RunStateMatrixASAP = false;
              startSM(); // Start the state matrix without waiting for an explicit 'R' command.
            }
          }
        }
      break;
      case 'R':  // Run State Machine
        startSM();
      break;
      case 'X':   // Exit state matrix and return data
        MatrixFinished = true;
        RunningStateMatrix = false;
        resetOutputs(); // Returns all lines to low by forcing final state
      break;
    } // End switch commandbyte
  } // End SerialUSB.available
  if (RunningStateMatrix) {
    if (firstLoop == 1) {
      firstLoop = 0;
      MatrixStartTimeMicros = sessionTimeMicros(); 
      timeBuffer.uint64[0] = MatrixStartTimeMicros;
      PC.writeByteArray(timeBuffer.byteArray,8); // Send trial-start timestamp (from micros() clock)
      SyncWrite();
      setStateOutputs(CurrentState); // Adjust outputs, global timers, serial codes and sync port for first state
    } else {
      //nCurrentEvents = 0;
      //CurrentEvent[0] = 254; // Event 254 = No event
      CurrentTime++;
      for (int i = BNCInputPos; i < nInputs; i++) {
          if (inputEnabled[i] && !inputOverrideState[i]) {
            inputState[i] = digitalReadDirect(InputCh[i]); 
          } 
      }
      // Determine if a handled condition occurred
      Ev = ConditionPos;
      for (int i = 0; i < nConditionsUsed; i++) {
        if (ConditionMatrix[CurrentState][i] != CurrentState) { // If this condition is handled
          if (inputState[ConditionChannels[i]] == ConditionValues[i]) {
            CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
          }
        }
        Ev++;
      }
      // Determine if a digital low->high or high->low transition event occurred
      Ev = DigitalInputPos;
      for (int i = BNCInputPos; i < nInputs; i++) {
          if (inputEnabled[i] == 1) {
              if ((inputState[i] == logicHigh[i]) && (lastInputState[i] == logicLow[i])) {
                lastInputState[i] = logicHigh[i]; CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
              }
          }
          Ev++;
          if (inputEnabled[i] == 1) {
              if ((inputState[i] == logicLow[i]) && (lastInputState[i] == logicHigh[i])) {
                lastInputState[i] = logicLow[i]; CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
              }
          }
          Ev++;
      }
      // Determine if a USB or hardware serial event occurred
      Ev = 0; Byte1 = 0;
      for (int i = 0; i < BNCInputPos; i++) {
        switch(InputHW[i]) {
          case 'U':
          if (usesUARTInputs) {
            switch(Byte1) {
              case 0:
                if (Module1.available() > 0) {
                  Byte2 = Module1.readByte();
                  if (Byte2 <= nModuleEvents[0]) {
                    CurrentEvent[nCurrentEvents] = Byte2 + Ev-1; nCurrentEvents++; 
                  }
                }
                Byte1++;
              break;
              case 1:
                if (Module2.available() > 0) {
                  Byte2 = Module2.readByte();
                  if (Byte2 <= nModuleEvents[1]) {
                    CurrentEvent[nCurrentEvents] = Byte2 + Ev-1; nCurrentEvents++; 
                  }
                }
                Byte1++;
              break;
              case 2:
                #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
                  if (Module3.available() > 0) {
                    Byte2 = Module3.readByte();
                    if (Byte2 <= nModuleEvents[2]) {
                      CurrentEvent[nCurrentEvents] = Byte2 + Ev-1; nCurrentEvents++; 
                    }
                  }
                #endif
                Byte1++;
              break;
              case 3:
                #if MACHINE_TYPE == 3
                  if (Module4.available() > 0) {
                    Byte2 = Module4.readByte();
                    if (Byte2 <= nModuleEvents[3]) {
                      CurrentEvent[nCurrentEvents] = Byte2 + Ev-1; nCurrentEvents++; 
                    }
                  }
                #endif
                Byte1++;
              break;
              case 4:
                #if MACHINE_TYPE == 3 && ETHERNET_COM == 0
                  if (Module5.available() > 0) {
                    Byte2 = Module5.readByte();
                    if (Byte2 <= nModuleEvents[4]) {
                      CurrentEvent[nCurrentEvents] = Byte2 + Ev-1; nCurrentEvents++; 
                    }
                  }
                #endif
                Byte1++;
              break;
            }
          }
          break;
          case 'X':
              if (SoftEvent < 255) {
                CurrentEvent[nCurrentEvents] = SoftEvent + Ev; nCurrentEvents++;
                SoftEvent = 255;
              }
          break;
        }
        Ev += nModuleEvents[i];
      }
      Ev = GlobalTimerStartPos;
      // Determine if a global timer expired
      for (int i = 0; i < nGlobalTimersUsed; i++) {
        if (GlobalTimersActive[i] == true) {
          if (CurrentTime >= GlobalTimerEnd[i]) {
            setGlobalTimerChannel(i, 0);
            GlobalTimersTriggered[i] = false;
            GlobalTimersActive[i] = false;
            if (GlobalTimerLoop[i] > 0) {
              if (GlobalTimerLoopCount[i] < GlobalTimerLoop[i])  {
                GlobalTimersTriggered[i] = true;
                GlobalTimerStart[i] = CurrentTime + GlobalTimerLoopIntervals[i];
                GlobalTimerEnd[i] = GlobalTimerStart[i] + GlobalTimers[i];
                if (SendGlobalTimerEvents[i]) {
                  CurrentEvent[nCurrentEvents] = Ev+MAX_GLOBAL_TIMERS; nCurrentEvents++;
                }
                if (GTUsingLoopCounter[i]) {
                  GlobalTimerLoopCount[i] += 1;
                }
              } else {
                if (SendGlobalTimerEvents[i]) {
                  CurrentEvent[nCurrentEvents] = Ev+MAX_GLOBAL_TIMERS; nCurrentEvents++;
                }
              }
            } else {
              CurrentEvent[nCurrentEvents] = Ev+MAX_GLOBAL_TIMERS; nCurrentEvents++;
            }
          }
        } else if (GlobalTimersTriggered[i] == true) {
          if (CurrentTime >= GlobalTimerStart[i]) {
            GlobalTimersActive[i] = true;
            if (GlobalTimerLoop[i]) {
              if (SendGlobalTimerEvents[i]) {
                CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
              }
            } else {
              CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
            }
            if (GlobalTimerOnsetTriggers[i] > 0) {
              for (int j = 0; j < nGlobalTimersUsed; j++) {
                if (bitRead(GlobalTimerOnsetTriggers[i], j)) {
                  if (j != i) {
                    triggerGlobalTimer(j);
                  }
                }
              }
            }
            setGlobalTimerChannel(i, 1);
          }
        }
        Ev++;
      }
      
      Ev = GlobalCounterPos;
      // Determine if a global event counter threshold was exceeded
      for (int x = 0; x < nGlobalCountersUsed; x++) {
        if (GlobalCounterAttachedEvents[x] < 254) {
          // Check for and handle threshold crossing
          if ((GlobalCounterCounts[x] == GlobalCounterThresholds[x]) && (GlobalCounterHandled[x] == false)) {
            CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
            GlobalCounterHandled[x] = true;
          }
          // Add current event to count (Crossing triggered on next cycle)
          for (int i = 0; i < nCurrentEvents; i++) {
            if (CurrentEvent[i] == GlobalCounterAttachedEvents[x]) {
              GlobalCounterCounts[x] = GlobalCounterCounts[x] + 1;
            }
          }
        }
        Ev++;
      }
      // Determine if a state timer expired
      Ev = TupPos;
      TimeFromStart = CurrentTime - StateStartTime;
      if ((TimeFromStart >= StateTimers[CurrentState]) && (MeaningfulStateTimer == true)) {
        CurrentEvent[nCurrentEvents] = Ev; nCurrentEvents++;
      }
      if (nCurrentEvents > maxCurrentEvents) { // Drop events beyond maxCurrentEvents
        nCurrentEvents = maxCurrentEvents;
      }
      // Now determine if a state transition should occur. The first event linked to a state transition takes priority.
      byte StateTransitionFound = 0; int i = 0; int CurrentColumn = 0;
      NewState = CurrentState;
      while ((!StateTransitionFound) && (i < nCurrentEvents)) {
        if (CurrentEvent[i] < GlobalTimerStartPos) {
          NewState = InputStateMatrix[CurrentState][CurrentEvent[i]];
        } else if (CurrentEvent[i] < GlobalTimerEndPos) {
          CurrentColumn = CurrentEvent[i] - GlobalTimerStartPos;
          NewState = GlobalTimerStartMatrix[CurrentState][CurrentColumn];
        } else if (CurrentEvent[i] < GlobalCounterPos) {
          CurrentColumn = CurrentEvent[i] - GlobalTimerEndPos;
          NewState = GlobalTimerEndMatrix[CurrentState][CurrentColumn];
        } else if (CurrentEvent[i] < ConditionPos) {
          CurrentColumn = CurrentEvent[i] - GlobalCounterPos;
          NewState = GlobalCounterMatrix[CurrentState][CurrentColumn];
        } else if (CurrentEvent[i] < TupPos) {
          CurrentColumn = CurrentEvent[i] - ConditionPos;
          NewState = ConditionMatrix[CurrentState][CurrentColumn];
        } else if (CurrentEvent[i] == TupPos) {
          NewState = StateTimerMatrix[CurrentState];
        }
        if (NewState != CurrentState) {
          StateTransitionFound = 1;
        }
        i++;
      }

      // Store timestamps of events captured in this cycle
      #if LIVE_TIMESTAMPS == 0
        #if MACHINE_TYPE == 2
          for (int i = 0; i < nCurrentEvents; i++) {
            CurrentTimeStamps.Uint32[i] = CurrentTime;
            nEvents++;
          }
          digitalWriteDirect(fRAMcs, LOW);
          digitalWriteDirect(fRAMhold, HIGH); // Resume logging
          SPI.transfer(CurrentTimeStamps.Bytes, nCurrentEvents * 4);
          digitalWriteDirect(fRAMhold, LOW); // Pause logging
          digitalWriteDirect(fRAMcs, HIGH);
        #else
          if ((nEvents + nCurrentEvents) < MaxTimestamps) {
            for (int x = 0; x < nCurrentEvents; x++) {
              Timestamps[nEvents] = CurrentTime;
              nEvents++;
            } 
          }
        #endif
      #endif
      // Write events captured to USB (if events were captured)
      if (nCurrentEvents > 0) {
        CurrentEventBuffer[1] = nCurrentEvents;
        for (int i = 2; i < nCurrentEvents+2; i++) {
          CurrentEventBuffer[i] = CurrentEvent[i-2];
        }
        #if LIVE_TIMESTAMPS == 0
          PC.writeByteArray(CurrentEventBuffer, nCurrentEvents+2);
        #else  
          typeBuffer.uint32 = CurrentTime;
          CurrentEventBuffer[nCurrentEvents+2] = typeBuffer.byteArray[0];
          CurrentEventBuffer[nCurrentEvents+3] = typeBuffer.byteArray[1];
          CurrentEventBuffer[nCurrentEvents+4] = typeBuffer.byteArray[2];
          CurrentEventBuffer[nCurrentEvents+5] = typeBuffer.byteArray[3];
          PC.writeByteArray(CurrentEventBuffer, nCurrentEvents+6);
        #endif   
      }
      nCurrentEvents = 0;
      CurrentEvent[0] = 254; // 254 = no event
      // Make state transition if necessary
      if (NewState != CurrentState) {
        if (NewState == nStates) {
          RunningStateMatrix = false;
          MatrixFinished = true;
        } else {
          if (SyncMode == 1) {
            SyncWrite();
          }
          StateStartTime = CurrentTime;
          if (using255BackSignal && (NewState == 255)) {
            CurrentStateTEMP = CurrentState;
            CurrentState = previousState;
            previousState = CurrentStateTEMP;
          } else {
            previousState = CurrentState;
            CurrentState = NewState;
          }
          setStateOutputs(CurrentState);
        }
      }
    } // End code to run after first loop
  }  else { // If not running state matrix
    
   
  } // End if not running state matrix
  if (MatrixFinished) {
    if (SyncMode == 0) {
      ResetSyncLine();
    } else {
      SyncWrite();
    }
    MatrixEndTimeMicros = sessionTimeMicros();
    resetOutputs();
// Send trial timing data back to computer
    serialByteBuffer[0] = 1; // Op Code for sending events
    serialByteBuffer[1] = 1; // Read one event
    serialByteBuffer[2] = 255; // Send Matrix-end code
    PC.writeByteArray(serialByteBuffer, 3);
    #if LIVE_TIMESTAMPS == 1
      timeBuffer.uint32[0] = CurrentTime;
      timeBuffer.uint32[1] = nCyclesCompleted;
      timeBuffer.uint64[1] = MatrixEndTimeMicros;
      PC.writeByteArray(timeBuffer.byteArray, 16);
      //PC.writeUint32(CurrentTime);
    #else
      PC.writeUint32(nCyclesCompleted);
      timeBuffer.uint64[0] = MatrixEndTimeMicros;
      PC.writeByteArray(timeBuffer.byteArray, 8);
    #endif
    
    #if LIVE_TIMESTAMPS == 0
      PC.writeUint16(nEvents);
      #if MACHINE_TYPE == 2
        // Return event times from fRAM IC
        digitalWriteDirect(fRAMhold, HIGH);
        digitalWriteDirect(fRAMcs, LOW);
        SPI.transfer(3); // Send read op code
        SPI.transfer(0); // Send address bytes
        SPI.transfer(0);
        SPI.transfer(0);
        uint16_t nFullBufferReads = 0; // A buffer array (SerialRelayBuffer) will store data read from RAM and dump it to USB
        uint16_t nRemainderBytes = 0;
        if (nEvents*4 > SerialRelayBufferSize) {
          nFullBufferReads = (unsigned long)(floor(((double)nEvents)*4 / (double)SerialRelayBufferSize));
        } else {
          nFullBufferReads = 0;
        }  
        for (int i = 0; i < nFullBufferReads; i++) { // Full buffer transfers; skipped if nFullBufferReads = 0
          SPI.transfer(SerialRelayBuffer, SerialRelayBufferSize);
          PC.writeByteArray(SerialRelayBuffer, SerialRelayBufferSize);
        }
        nRemainderBytes = (nEvents*4)-(nFullBufferReads*SerialRelayBufferSize);
        if (nRemainderBytes > 0) {
          SPI.transfer(SerialRelayBuffer, nRemainderBytes);
          PC.writeByteArray(SerialRelayBuffer, nRemainderBytes);   
        }
        digitalWriteDirect(fRAMcs, HIGH);
      #else
        PC.writeUint32Array(Timestamps, nEvents);
      #endif  
    #endif   
    MatrixFinished = false;
    if (smaReady2Load) { // If the next trial's state matrix was loaded to the serial buffer during the trial
      loadStateMatrix();
      smaReady2Load = false;
    }
    updateStatusLED(0);
    updateStatusLED(2);
    if (RunStateMatrixASAP) {
      RunStateMatrixASAP = false;
      if (newSMATransmissionStarted){
          if (smaTransmissionConfirmed) {
            startSM();
          } else {
            runFlag = true; // Set run flag to true; new SM will start as soon as its transmission completes
          }
      } else {
        startSM();
      }
    }
  } // End Matrix finished
  nCyclesCompleted++;
} // End timer handler

void ResetSyncLine() {
  if (!usesSPISync) {
    SyncState = 0;
    if (SyncChannelOriginalType == 'P') {
      analogWrite(SyncChannelHW, SyncState);
    } else {
      digitalWriteDirect(SyncChannelHW, SyncState);     
    }
  }
}
void SyncWrite() {
  if (!usesSPISync) {
    if (syncOn) { ;
      if (SyncState == 0) {
        SyncState = 1;
      } else {
        SyncState = 0;
      }
      if (SyncChannelOriginalType == 'P') {
        analogWrite(SyncChannelHW, SyncState*255);
      } else {
        digitalWriteDirect(SyncChannelHW, SyncState);     
      }
    }
  }
}
void SyncRegWrite(int value) {
  if (usesSPISync) {
    SPI.transfer(value);
    digitalWriteDirect(SyncRegisterLatch,HIGH);
    digitalWriteDirect(SyncRegisterLatch,LOW);
  }
}
void updateStatusLED(int Mode) {
  if (STATUS_LED_ENABLED) {
    CurrentTime = millis();
    switch (Mode) {
      case 0: {
          analogWrite(RedLEDPin, 0);
          digitalWriteDirect(GreenLEDPin, 0);
          analogWrite(BlueLEDPin, 0);
        } break;
      case 1: { // Waiting for matrix
          if (connectionState == 0) {
            if (CurrentTime > NextLEDBrightnessAdjustTime) {
              NextLEDBrightnessAdjustTime = CurrentTime + LEDBrightnessAdjustInterval;
              if (LEDBrightnessAdjustDirection == 1) {
                if (LEDBrightness < 255) {
                  LEDBrightness = LEDBrightness + 1;
                } else {
                  LEDBrightnessAdjustDirection = 0;
                }
              }
              if (LEDBrightnessAdjustDirection == 0) {
                if (LEDBrightness > 0) {
                  LEDBrightness = LEDBrightness - 1;
                } else {
                  LEDBrightnessAdjustDirection = 2;
                }
              }
              if (LEDBrightnessAdjustDirection == 2) {
                NextLEDBrightnessAdjustTime = CurrentTime + 500;
                LEDBrightnessAdjustDirection = 1;
              }
              analogWrite(BlueLEDPin, LEDBrightness);
            }
          }
        } break;
      case 2: {
          analogWrite(BlueLEDPin, 0);
          digitalWriteDirect(GreenLEDPin, 1);
        } break;
      case 3: {
          digitalWrite(GreenLEDPin, HIGH);
          analogWrite(BlueLEDPin, 0);
          analogWrite(RedLEDPin, 128);
        } break;
    }
  }
}

void setStateOutputs(byte State) { 
  uint16_t CurrentTimer = 0; // Used when referring to the timer currently being triggered
  byte CurrentCounter = 0; // Used when referring to the counter currently being reset
  byte thisChannel = 0;
  byte thisMessage = 0;
  byte nMessageBytes = 0;
  boolean reqValveUpdate = false;
  SyncRegWrite((State+1)); // If firmware 0.5 or 0.6, writes current state code to shift register
  
  // Cancel global timers
  CurrentTimer = smGlobalTimerCancel[State];
  if (CurrentTimer > 0) {
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      if (bitRead(CurrentTimer, i)) {
        if (GlobalTimersActive[i]) {
          GlobalTimersTriggered[i] = false;
          GlobalTimersActive[i] = false;
          setGlobalTimerChannel(i, 0);
          CurrentEvent[nCurrentEvents] = i+GlobalTimerEndPos; nCurrentEvents++;
        } else if (GlobalTimersTriggered[i]) {
          GlobalTimersTriggered[i] = false;
        }
      }
    }
  }
  
  // Trigger global timers
  CurrentTimer = smGlobalTimerTrig[State];  
  if (CurrentTimer > 0) {
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      if (bitRead(CurrentTimer, i)) {
        triggerGlobalTimer(i);
      }
    }
  }
  
  // Update output channels
  for (int i = 0; i < nOutputs; i++) {
      switch(OutputHW[i]) {
        case 'U':
        thisMessage = OutputStateMatrix[State][i];
        if (thisMessage > 0) {
          nMessageBytes = SerialMessage_nBytes[thisMessage][thisChannel];
          for (int i = 0; i < nMessageBytes; i++) {
             serialByteBuffer[i] = SerialMessageMatrix[thisMessage][thisChannel][i];
          }
          switch(thisChannel) {
            case 0:
              Module1.writeByteArray(serialByteBuffer, nMessageBytes);
            break;
            case 1:
              Module2.writeByteArray(serialByteBuffer, nMessageBytes);
            break;
            #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
              case 2:
                Module3.writeByteArray(serialByteBuffer, nMessageBytes);
              break;
            #endif
            #if MACHINE_TYPE == 3
              case 3:
                Module4.writeByteArray(serialByteBuffer, nMessageBytes);
              break;
              #if ETHERNET_COM == 0
                case 4:
                  Module5.writeByteArray(serialByteBuffer, nMessageBytes);
                break;
              #endif
            #endif
          }
        }
        thisChannel++;
        break;
        case 'X':
        if (OutputStateMatrix[State][i] > 0) {
          serialByteBuffer[0] = 2; // Code for MATLAB to receive soft-code byte
          serialByteBuffer[1] = OutputStateMatrix[State][i]; // Soft code byte
          PC.writeByteArray(serialByteBuffer, 2);
        }
        break;
        case 'D':
        case 'B':
        case 'W':
          if (outputOverrideState[i] == 0) {
            digitalWriteDirect(OutputCh[i], OutputStateMatrix[State][i]); 
          }
        case 'V':
          if (outputOverrideState[i] == 0) {
            if (usesSPIValves) {
              outputState[i] = OutputStateMatrix[State][i];
              reqValveUpdate = true;
            } else {
              digitalWriteDirect(OutputCh[i], OutputStateMatrix[State][i]); 
            }
          }
        break;
        case 'P':
          if (outputOverrideState[i] == 0) {   
            #if MACHINE_TYPE < 3
              //Arduino Due PERFORMANCE HACK: the next line is equivalent to: analogWrite(OutputCh[i], OutputStateMatrix[State][i]); 
              PWMC_SetDutyCycle(PWM_INTERFACE, PWMChannel[i], OutputStateMatrix[State][i]);
            #else
              analogWrite(OutputCh[i], OutputStateMatrix[State][i]); 
            #endif
          }
        break;
     }
  }
  // Update valves
  if (reqValveUpdate) {
    valveWrite();
    reqValveUpdate = false;
  }
  // Reset event counters
  CurrentCounter = smGlobalCounterReset[State];
  if (CurrentCounter > 0) {
    CurrentCounter = CurrentCounter - 1; // Convert to 0 index
    GlobalCounterCounts[CurrentCounter] = 0;
  }

  // Enable state timer only if handled
  if (StateTimerMatrix[State] != State) {
    MeaningfulStateTimer = true;
  } else {
    MeaningfulStateTimer = false;
  }
}

void triggerGlobalTimer(byte timerID) {
  GlobalTimersTriggered[timerID] = true;
  GlobalTimerStart[timerID] = CurrentTime;
  if (GlobalTimerOnsetDelays[timerID] > 0){
    GlobalTimerStart[timerID] += GlobalTimerOnsetDelays[timerID];
  } else {
    if (!GlobalTimersActive[timerID]) {
      GlobalTimersActive[timerID] = true;
      setGlobalTimerChannel(timerID, 1);
      if (GlobalTimerOnsetTriggers[timerID] > 0) {
        for (int j = 0; j < nGlobalTimersUsed; j++) {
          if (bitRead(GlobalTimerOnsetTriggers[timerID], j)) {
            if (j != timerID) {
              triggerGlobalTimer(j);
            }
          }
        }
      }
      CurrentEvent[nCurrentEvents] = timerID+GlobalTimerStartPos; nCurrentEvents++;
    }
  }
  GlobalTimerEnd[timerID] = GlobalTimerStart[timerID] + GlobalTimers[timerID];
  if (GTUsingLoopCounter[timerID]) {
    GlobalTimerLoopCount[timerID] = 1;
  } 
}

void resetOutputs() {
  for (int i = 0; i < nOutputs; i++) {
    switch (OutputHW[i]) {
      case 'B':
      case 'W':
      case 'V':
        digitalWriteDirect(OutputCh[i], 0);
        outputOverrideState[i] = 0; 
        outputState[i] = 0;
      break;
      case 'P':
        analogWrite(OutputCh[i], 0); 
        outputOverrideState[i] = 0; 
      break;
    }
  }
  valveWrite();
  for (int i = 0; i < MAX_GLOBAL_TIMERS; i++) {
    GlobalTimersTriggered[i] = false;
    GlobalTimersActive[i] = false;
    GlobalTimerLoopCount[i] = 0;
  }
  for (int i = 0; i < MAX_GLOBAL_COUNTERS; i++) {  
    GlobalCounterCounts[i] = 0;
    GlobalCounterHandled[i] = false;
  }
  MeaningfulStateTimer = false;
}

uint64_t sessionTimeMicros() {
   uint32_t currentTimeMicros = 0;
   uint64_t sessionTime = 0;
   currentTimeMicros = micros();
   sessionTime = ((uint64_t)currentTimeMicros + ((uint64_t)nMicrosRollovers*4294967295)) - sessionStartTimeMicros;
   return sessionTime;
}

void resetSessionClock() {
    sessionStartTimeMicros = micros();
    nMicrosRollovers = 0;
}

void resetSerialMessages() {
  for (int i = 0; i < MaxStates; i++) {
    for (int j = 0; j < nSerialChannels-1; j++) {
      SerialMessageMatrix[i][j][0] = i;
      SerialMessage_nBytes[i][j] = 1;
    }
  }
}

void valveWrite() {
  byte value = 0;
  for (int i = ValvePos; i < nOutputs; i++) {
    if (outputState[i] == 1) {
      bitSet(value, i-ValvePos);
    }
  }
  SPI.transfer(value);
  digitalWriteDirect(valveCSChannel, HIGH);
  digitalWriteDirect(valveCSChannel, LOW);
}

void digitalWriteDirect(int pin, boolean val) { // >10x Faster than digitalWrite(), specific to Arduino Due
  #if MACHINE_TYPE < 3
    if (val) g_APinDescription[pin].pPort -> PIO_SODR = g_APinDescription[pin].ulPin;
    else    g_APinDescription[pin].pPort -> PIO_CODR = g_APinDescription[pin].ulPin;
  #else
    digitalWrite(pin, val);
  #endif
}

byte digitalReadDirect(int pin) { // >10x Faster than digitalRead(), specific to Arduino Due
  #if MACHINE_TYPE < 3
    return !!(g_APinDescription[pin].pPort -> PIO_PDSR & g_APinDescription[pin].ulPin);
  #else
    return digitalRead(pin);
  #endif
}

void setGlobalTimerChannel(byte timerChan, byte op) {
  byte thisChannel = 0; 
  byte thisMessage = 0;
  byte nMessageBytes = 0;
  thisChannel = GlobalTimerChannel[timerChan];
  switch (OutputHW[thisChannel]) {
    case 'U': // UART
      if (op == 1) {
        thisMessage = GlobalTimerOnMessage[timerChan];
      } else {
        thisMessage = GlobalTimerOffMessage[timerChan];
      }
      if (thisMessage < 254) {
        nMessageBytes = SerialMessage_nBytes[thisMessage][thisChannel];
          for (int i = 0; i < nMessageBytes; i++) {
             serialByteBuffer[i] = SerialMessageMatrix[thisMessage][thisChannel][i];
          }
        switch (thisChannel) {
          case 0:
            Module1.writeByteArray(serialByteBuffer, nMessageBytes);
          break;
          case 1:
            Module2.writeByteArray(serialByteBuffer, nMessageBytes);
          break;
          #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
            case 2:
              Module3.writeByteArray(serialByteBuffer, nMessageBytes);
            break;
          #endif
          #if MACHINE_TYPE == 3
            case 3:
              Module4.writeByteArray(serialByteBuffer, nMessageBytes);
            break;
            #if ETHERNET_COM == 0
              case 4:
                Module5.writeByteArray(serialByteBuffer, nMessageBytes);
              break;
            #endif
          #endif
        }
      }
    break;
    case 'B': // Digital IO (BNC, Wire, Digital, Valve-line)
    case 'W':
    case 'D':
      if (op == 1) {
        digitalWriteDirect(OutputCh[thisChannel], HIGH);
        outputOverrideState[thisChannel] = 1;
      } else {
        digitalWriteDirect(OutputCh[thisChannel], LOW);
        outputOverrideState[thisChannel] = 0;
      }
    break;
    case 'V':
      if (usesSPIValves) {
        outputState[thisChannel] = op;
        valveWrite();
      } else {
        if (op == 1) {
          digitalWriteDirect(OutputCh[thisChannel], HIGH);
        } else {
          digitalWriteDirect(OutputCh[thisChannel], LOW);
        }
      }
      outputOverrideState[thisChannel] = op;
    break;
    case 'P': // Port (PWM / LED)
      if (op == 1) {
        analogWrite(OutputCh[thisChannel], GlobalTimerOnMessage[timerChan]);
        outputOverrideState[thisChannel] = 1;
      } else {
        analogWrite(OutputCh[thisChannel], 0);
        outputOverrideState[thisChannel] = 0;
      }
    break;
  }
  inputState[nInputs+timerChan] = op;
}

void relayModuleInfo(ArCOM serialCOM, byte moduleID) {
  boolean moduleFound = false;
  boolean setBaudRate = false;
  uint32_t newBaudRate = 0;
  if (serialCOM.available() > 0) {
    Byte1 = serialCOM.readByte();
    if (Byte1 == 'A') { // A = Acknowledge; this is most likely a module
      if (serialCOM.available() > 3) {
        moduleFound = true;
        PC.writeByte(1); // Module detected
        for (int i = 0; i < 4; i++) { // Send firmware version
          PC.writeByte(serialCOM.readByte());
        }
        nBytes = serialCOM.readByte(); // Length of module name
        PC.writeByte(nBytes);
        for (int i = 0; i < nBytes; i++) { // Transfer module name
          PC.writeByte(serialCOM.readByte());
        }
        Byte1 = serialCOM.readByte(); // 1 if more module info follows, 0 if not
        if (Byte1 == 1) { // Optional: module can return additional info with an op code scheme
          while(Byte1 == 1) {
            PC.writeByte(1); // indicate that more info is coming for this module
            CommandByte = serialCOM.readByte();
            switch (CommandByte) {
              case '#': // Number of behavior events to reserve for the module
                PC.writeByte('#');
                PC.writeByte(serialCOM.readByte());
              break;
              case 'E': // Event name strings (replace default event names: ModuleName1_1, etc)
                PC.writeByte('E');
                Byte2 = serialCOM.readByte(); // nEvent names to transmit
                PC.writeByte(Byte2);
                for (int i = 0; i < Byte2; i++) {
                  Byte3 = serialCOM.readByte(); // Length of event name (in characters)
                  PC.writeByte(Byte3);
                  serialCOM.readByteArray(SerialRelayBuffer, Byte3);
                  PC.writeByteArray(SerialRelayBuffer, Byte3);
                }
              break;
            }
            Byte1 = serialCOM.readByte(); // 1 if more module info follows, 0 if not
          }
          PC.writeByte(0); // indicate that no more info is coming for this module 
        } else {
          PC.writeByte(0); // indicate that no more info is coming for this module 
        }
      }
    }
  }
  if (!moduleFound) {
    PC.writeByte(0); // Module not detected
  }
}

void disableModuleRelays() {
  for (int i = 0; i < nSerialChannels; i++) { // Shut off all other channels
    UARTrelayMode[Byte1] = false;
  }
}

void relayModuleBytes() {
  for (int i = 0; i < nSerialChannels; i++) { // If relay mode is on, return any incoming module bytes to MATLAB/Python
      if (UARTrelayMode[i]) {
        switch(i) {
          case 0:
            Byte3 = Module1.available();
            if (Byte3>0) { 
              for (int j = 0; j < Byte3; j++) {
                 PC.writeByte(Module1.readByte());       
              }
            }
          break;
          case 1:
            Byte3 = Module2.available();
            if (Byte3>0) { 
              for (int j = 0; j < Byte3; j++) {
                 PC.writeByte(Module2.readByte());       
              }
            }
          break;
          #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
            case 2:
              Byte3 = Module3.available();
              if (Byte3>0) { 
                for (int j = 0; j < Byte3; j++) {
                   PC.writeByte(Module3.readByte());       
                }
              }
            break;
          #endif
          #if MACHINE_TYPE == 3
            case 3:
                Byte3 = Module4.available();
                if (Byte3>0) { 
                  for (int j = 0; j < Byte3; j++) {
                     PC.writeByte(Module4.readByte());       
                  }
                }
            break;
            #if ETHERNET_COM == 0
              case 4:
                Byte3 = Module5.available();
                if (Byte3>0) { 
                  for (int j = 0; j < Byte3; j++) {
                     PC.writeByte(Module5.readByte());       
                  }
                }
              break;
            #endif
          #endif
        }
      }
    }
}

void clearSerialBuffers() {
  Byte1 = 0;
  for (int i = 0; i < BNCInputPos; i++) {
      switch (InputHW[i]) {
        case 'U': 
            switch(Byte1) {
              case 0:
                while (Module1.available() > 0) {
                  Module1.readByte(); Byte1++;
                }
              break;
              case 1:
                while (Module2.available() > 0) {
                  Module2.readByte(); Byte1++;
                }
              break;
              #if MACHINE_TYPE == 2 || MACHINE_TYPE == 3
                case 2:
                  while (Module3.available() > 0) {
                    Module3.readByte(); Byte1++;
                  }
                break;
              #endif
              #if MACHINE_TYPE == 3
                case 3:
                  while (Module4.available() > 0) {
                    Module4.readByte(); Byte1++;
                  }
                break;
                #if ETHERNET_COM == 0
                  case 4:
                    while (Module5.available() > 0) {
                      Module5.readByte(); Byte1++;
                    }
                  break;
                #endif
              #endif
            }
       break;
     }
  }
}

void loadStateMatrix() { // Loads a state matrix from the serial buffer into the relevant local variables
  #if MACHINE_TYPE > 1
    nStates = StateMatrixBuffer[0];
    nGlobalTimersUsed = StateMatrixBuffer[1];
    nGlobalCountersUsed = StateMatrixBuffer[2];
    nConditionsUsed = StateMatrixBuffer[3];
  #else
    nStates = PC.readByte();
    nGlobalTimersUsed = PC.readByte();
    nGlobalCountersUsed = PC.readByte();
    nConditionsUsed = PC.readByte();
  #endif
  bufferPos = 4; // Current position in serial relay buffer
  for (int x = 0; x < nStates; x++) { // Set matrix to default
    StateTimerMatrix[x] = 0;
    for (int y = 0; y < InputMatrixSize; y++) {
      InputStateMatrix[x][y] = x;
    }
    for (int y = 0; y < OutputMatrixSize; y++) {
      OutputStateMatrix[x][y] = 0;
    }
    smGlobalTimerTrig[x] = 0;
    smGlobalTimerCancel[x] = 0;
    smGlobalCounterReset[x] = 0;
    for (int y = 0; y < MAX_GLOBAL_TIMERS; y++) {
      GlobalTimerStartMatrix[x][y] = x;
    }
    for (int y = 0; y < MAX_GLOBAL_TIMERS; y++) {
      GlobalTimerEndMatrix[x][y] = x;
    }
    for (int y = 0; y < MAX_GLOBAL_COUNTERS; y++) {
      GlobalCounterMatrix[x][y] = x;
    }
    for (int y = 0; y < MAX_CONDITIONS; y++) {
      ConditionMatrix[x][y] = x;
    }
  }
  #if MACHINE_TYPE > 1 // Bpod 0.7+; Read state matrix from RAM buffer
    for (int x = 0; x < nStates; x++) { // Get State timer matrix
      StateTimerMatrix[x] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int x = 0; x < nStates; x++) { // Get Input Matrix differences
      nOverrides = StateMatrixBuffer[bufferPos]; bufferPos++;
      for (int y = 0; y<nOverrides; y++) {
        col = StateMatrixBuffer[bufferPos]; bufferPos++;
        val = StateMatrixBuffer[bufferPos]; bufferPos++;
        InputStateMatrix[x][col] = val;
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Output Matrix differences
      nOverrides = StateMatrixBuffer[bufferPos]; bufferPos++;
      for (int y = 0; y<nOverrides; y++) {
        col = StateMatrixBuffer[bufferPos]; bufferPos++;
        val = StateMatrixBuffer[bufferPos]; bufferPos++;
        OutputStateMatrix[x][col] = val;
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Global Timer Start Matrix differences
      nOverrides = StateMatrixBuffer[bufferPos]; bufferPos++;
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = StateMatrixBuffer[bufferPos]; bufferPos++;
          val = StateMatrixBuffer[bufferPos]; bufferPos++;
          GlobalTimerStartMatrix[x][col] = val;
        }
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Global Timer End Matrix differences
      nOverrides = StateMatrixBuffer[bufferPos]; bufferPos++;
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = StateMatrixBuffer[bufferPos]; bufferPos++;
          val = StateMatrixBuffer[bufferPos]; bufferPos++;
          GlobalTimerEndMatrix[x][col] = val;
        }
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Global Counter Matrix differences
      nOverrides = StateMatrixBuffer[bufferPos]; bufferPos++;
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = StateMatrixBuffer[bufferPos]; bufferPos++;
          val = StateMatrixBuffer[bufferPos]; bufferPos++;
          GlobalCounterMatrix[x][col] = val;
        }
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Condition Matrix differences
      nOverrides = StateMatrixBuffer[bufferPos]; bufferPos++;
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = StateMatrixBuffer[bufferPos]; bufferPos++;
          val = StateMatrixBuffer[bufferPos]; bufferPos++;
          ConditionMatrix[x][col] = val;
        }
      }
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      GlobalTimerChannel[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      GlobalTimerOnMessage[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      GlobalTimerOffMessage[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      GlobalTimerLoop[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
      if (GlobalTimerLoop[i] > 1) {
        GTUsingLoopCounter[i] = true;
      } else {
        GTUsingLoopCounter[i] = false;
      }
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      SendGlobalTimerEvents[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nGlobalCountersUsed; i++) {
      GlobalCounterAttachedEvents[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nConditionsUsed; i++) {
      ConditionChannels[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nConditionsUsed; i++) {
      ConditionValues[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }
    for (int i = 0; i < nStates; i++) {
      smGlobalCounterReset[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
    }

    #if GLOBALTIMER_TRIG_BYTEWIDTH == 1
        for (int i = 0; i < nStates; i++) {
          smGlobalTimerTrig[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
        }
        for (int i = 0; i < nStates; i++) {
          smGlobalTimerCancel[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
        }
        for (int i = 0; i < nGlobalTimersUsed; i++) {
          GlobalTimerOnsetTriggers[i] = StateMatrixBuffer[bufferPos]; bufferPos++;
        }
    #elif GLOBALTIMER_TRIG_BYTEWIDTH == 2
        for (int i = 0; i < nStates; i++) {
          typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
          smGlobalTimerTrig[i] = typeBuffer.uint16;
        }
        for (int i = 0; i < nStates; i++) {
          typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
          smGlobalTimerCancel[i] = typeBuffer.uint16;
        }
        for (int i = 0; i < nGlobalTimersUsed; i++) {
          typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
          GlobalTimerOnsetTriggers[i] = typeBuffer.uint16;
        }
     #elif GLOBALTIMER_TRIG_BYTEWIDTH == 4
        for (int i = 0; i < nStates; i++) {
          typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
          smGlobalTimerTrig[i] = typeBuffer.uint32;
        }
        for (int i = 0; i < nStates; i++) {
          typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
          smGlobalTimerCancel[i] = typeBuffer.uint32;
        }
        for (int i = 0; i < nGlobalTimersUsed; i++) {
          typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
          typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
          GlobalTimerOnsetTriggers[i] = typeBuffer.uint32;
        }
    #endif
    for (int i = 0; i < nStates; i++) {
      typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
      StateTimers[i] = typeBuffer.uint32;
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
      GlobalTimers[i] = typeBuffer.uint32;
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
      GlobalTimerOnsetDelays[i] = typeBuffer.uint32;
    }
    for (int i = 0; i < nGlobalTimersUsed; i++) {
      typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
      GlobalTimerLoopIntervals[i] = typeBuffer.uint32;
    }
    for (int i = 0; i < nGlobalCountersUsed; i++) {
      typeBuffer.byteArray[0] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[1] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[2] = StateMatrixBuffer[bufferPos]; bufferPos++;
      typeBuffer.byteArray[3] = StateMatrixBuffer[bufferPos]; bufferPos++;
      GlobalCounterThresholds[i] = typeBuffer.uint32;
    }
  #else // Bpod 0.5; Read state matrix from serial port
    for (int x = 0; x < nStates; x++) { // Get State timer matrix
      StateTimerMatrix[x] = PC.readByte();
    }
    for (int x = 0; x < nStates; x++) { // Get Input Matrix differences
      nOverrides = PC.readByte();
      for (int y = 0; y<nOverrides; y++) {
        col = PC.readByte();
        val = PC.readByte();
        InputStateMatrix[x][col] = val;
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Output Matrix differences
      nOverrides = PC.readByte();
      for (int y = 0; y<nOverrides; y++) {
        col = PC.readByte();
        val = PC.readByte();
        OutputStateMatrix[x][col] = val;
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Global Timer Start Matrix differences
      nOverrides = PC.readByte();
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = PC.readByte();
          val = PC.readByte();
          GlobalTimerStartMatrix[x][col] = val;
        }
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Global Timer End Matrix differences
      nOverrides = PC.readByte();
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = PC.readByte();
          val = PC.readByte();
          GlobalTimerEndMatrix[x][col] = val;
        }
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Global Counter Matrix differences
      nOverrides = PC.readByte();
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = PC.readByte();
          val = PC.readByte();
          GlobalCounterMatrix[x][col] = val;
        }
      }
    }
    for (int x = 0; x < nStates; x++) { // Get Condition Matrix differences
      nOverrides = PC.readByte();
      if (nOverrides > 0) {
        for (int y = 0; y<nOverrides; y++) {
          col = PC.readByte();
          val = PC.readByte();
          ConditionMatrix[x][col] = val;
        }
      }
    }
    if (nGlobalTimersUsed > 0) {
      PC.readByteArray(GlobalTimerChannel, nGlobalTimersUsed); // Get output channels of global timers
      PC.readByteArray(GlobalTimerOnMessage, nGlobalTimersUsed); // Get serial messages to trigger on timer start
      PC.readByteArray(GlobalTimerOffMessage, nGlobalTimersUsed); // Get serial messages to trigger on timer end
      PC.readByteArray(GlobalTimerLoop, nGlobalTimersUsed); // Get global timer loop state (true/false)
      for (int i = 0; i < nGlobalTimersUsed; i++) {
        if (GlobalTimerLoop[i] > 1) {
          GTUsingLoopCounter[i] = true;
        } else {
          GTUsingLoopCounter[i] = false;
        }
      }
      PC.readByteArray(SendGlobalTimerEvents, nGlobalTimersUsed); // Send global timer events (enabled/disabled)
    }
    if (nGlobalCountersUsed > 0) {
      PC.readByteArray(GlobalCounterAttachedEvents, nGlobalCountersUsed); // Get global counter attached events
    }
    if (nConditionsUsed > 0) {
      PC.readByteArray(ConditionChannels, nConditionsUsed); // Get condition channels
      PC.readByteArray(ConditionValues, nConditionsUsed); // Get condition values
    }
    PC.readByteArray(smGlobalCounterReset, nStates);
    #if GLOBALTIMER_TRIG_BYTEWIDTH == 1
        PC.readByteArray(smGlobalTimerTrig, nStates);
        PC.readByteArray(smGlobalTimerCancel, nStates);
        PC.readByteArray(GlobalTimerOnsetTriggers, nGlobalTimersUsed);
    #elif GLOBALTIMER_TRIG_BYTEWIDTH == 2
        PC.readUint16Array(smGlobalTimerTrig, nStates);
        PC.readUint16Array(smGlobalTimerCancel, nStates);
        PC.readUint16Array(GlobalTimerOnsetTriggers, nGlobalTimersUsed);
    #elif GLOBALTIMER_TRIG_BYTEWIDTH == 4
        PC.readUint32Array(smGlobalTimerTrig, nStates);
        PC.readUint32Array(smGlobalTimerCancel, nStates);
        PC.readUint32Array(GlobalTimerOnsetTriggers, nGlobalTimersUsed);
    #endif
    
    PC.readUint32Array(StateTimers, nStates); // Get state timers
    if (nGlobalTimersUsed > 0) {
      PC.readUint32Array(GlobalTimers, nGlobalTimersUsed); // Get global timers
      PC.readUint32Array(GlobalTimerOnsetDelays, nGlobalTimersUsed); // Get global timer onset delays
      PC.readUint32Array(GlobalTimerLoopIntervals, nGlobalTimersUsed); // Get loop intervals
    }
    if (nGlobalCountersUsed > 0) {
      PC.readUint32Array(GlobalCounterThresholds, nGlobalCountersUsed); // Get global counter event count thresholds
    }
    smaTransmissionConfirmed = true;
  #endif
}
void startSM() {  
  if (newSMATransmissionStarted){
      if (smaTransmissionConfirmed) {
        PC.writeByte(1);
      } else {
        PC.writeByte(0);
      }
      newSMATransmissionStarted = false;
  }
  updateStatusLED(3);
  NewState = 0;
  previousState = 0;
  CurrentState = 0;
  nEvents = 0;
  SoftEvent = 255; // No event
  MatrixFinished = false;
  #if LIVE_TIMESTAMPS == 0
    if (usesFRAM) {
      // Initialize fRAM
      SPI.beginTransaction(SPISettings(40000000, MSBFIRST, SPI_MODE0));
      digitalWriteDirect(fRAMcs, LOW);
      SPI.transfer(6); // Send Write enable code
      digitalWriteDirect(fRAMcs, HIGH); // Must go high afterwards to enable writes
      delayMicroseconds(10);
      digitalWriteDirect(fRAMcs, LOW);
      SPI.transfer(2); // Send write op code
      SPI.transfer(0); // Send address bytes
      SPI.transfer(0);
      SPI.transfer(0);
      digitalWriteDirect(fRAMhold, LOW); // Pause logging
      digitalWriteDirect(fRAMcs, HIGH);
    }
  #endif
  // Reset event counters
  for (int i = 0; i < MAX_GLOBAL_COUNTERS; i++) {
    GlobalCounterCounts[i] = 0;
    GlobalCounterHandled[i] = false;
  }
  // Read initial state of sensors
  for (int i = BNCInputPos; i < nInputs; i++) {
    if (inputEnabled[i] == 1) {
      inputState[i] =  digitalReadDirect(InputCh[i]);
      lastInputState[i] = inputState[i];
    } else {
      inputState[i] = logicLow[i];
      lastInputState[i] = logicLow[i];
    }
    inputOverrideState[i] = false;
  }
  for (int i = nInputs; i < nInputs+MAX_GLOBAL_TIMERS; i++) { // Clear global timer virtual lines
    inputState[i] = 0;
  }
  clearSerialBuffers();
  // Reset timers
  StateStartTime = 0;
  CurrentTime = 0;
  nCurrentEvents = 0;
  nCyclesCompleted = 0;
  CurrentEvent[0] = 254; // 254 = no event
  RunningStateMatrix = 1;
  firstLoop = 1;
}

