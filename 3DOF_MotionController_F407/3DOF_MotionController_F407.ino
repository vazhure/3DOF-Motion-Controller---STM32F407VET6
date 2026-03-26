// 3DOF Motion Controller - STM32F407VET6
// V1.4 - Fast GPIO for stepper control
//
// V1.4 Changes:
// 1. Fast GPIO macros for stepper pins (10-20x faster than digitalWrite)
// 2. Direct BSRR register access in ISR for minimum jitter
// 3. Separate FastGPIO.h file for reusable fast GPIO functions
//
// V1.3 Changes:
// 1. Reassign LIMIT PINS to PD0,PD1,PD4,PD5 due Timer3 / SPI2 conflict
//
// V1.2 Changes:
// 1. SPI Flash storage fix. No hardware SPI conflict with Timer2/Timer3
//
// V1.1 Changes:
// 1. SPI Flash storage for persistent settings (Winbond W25Q16)
// 2. PID controller with anti-windup
// 3. Non-blocking homing for all axes
// 4. Parallel axis motion (4 independent timers)
// 5. Compatible with Plugin.cs protocol
//
// Board: STM32F407VET6 (Black Pill or similar)
// CPU Speed: 168MHz
// Upload method: STLink
// Optimize: Fast (-O1)
//
// Email: v.azhure@gmail.com
// Modified: 2025-03-26

//#define DEBUG

#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include "FastGPIO.h"
#include "SpiFlashStorage.h"

// ============================================================================
// PACKED STRUCTURES (Compatible with Plugin.cs)
// ============================================================================
#pragma pack(push, 1)

enum MODE : uint8_t {
  UNKNOWN = 0,
  CONNECTED,
  DISABLED,
  HOMEING,
  PARKING,
  READY,
  ALARM
};

enum HOME_SUBSTATE : uint8_t {
  HOME_IDLE = 0,
  HOME_SEEKING_LIMIT,
  HOME_RETRACTING,
  HOME_MOVING_CENTER,
  HOME_DONE
};

enum COMMAND : uint8_t {
  CMD_HOME = 0x00,
  CMD_MOVE = 0x01,
  CMD_SET_SPEED = 0x02,
  CMD_DISABLE = 0x03,
  CMD_ENABLE = 0x04,
  CMD_GET_STATE = 0x05,
  CMD_CLEAR_ALARM = 0x06,
  CMD_PARK = 0x07,
  SET_ALARM = 0x08,
  CMD_SET_SLOW_SPEED = 0x09,
  CMD_SET_ACCEL = 0x0A,
  CMD_MOVE_SH = 0x0B,
  CMD_SET_PID_KP = 0x0C,
  CMD_SET_PID_KI = 0x0D,
  CMD_SET_PID_KD = 0x0E,
  CMD_SET_PID_KS = 0x0F,
  CMD_SET_PID_ENABLE = 0x10,
  CMD_SET_PID_BLEND = 0x11,
  CMD_GET_PID_STATE = 0x12,
  CMD_STORE_PID = 0x13,
  CMD_RESTORE_PID = 0x14
};

enum DEVICE_FLAGS : uint8_t {
  NONE = 0,
  STATE_ON_LIMIT_SWITCH = 1,
  STATE_HOMED = 1 << 1,
};

enum PID_FLAGS : uint8_t {
  PID_NONE = 0,
  PID_ENABLED = 1,
  PID_MASTER_SYNC = 1 << 1,
  PID_DIAG_ENABLED = 1 << 2,
};

struct STATE {
  uint8_t mode;            // 1
  uint8_t flags;           // 1
  uint16_t speedMMperSEC;  // 2
  int32_t currentpos;      // 4
  int32_t targetpos;       // 4
  int32_t min;             // 4
  int32_t max;             // 4
};                         // = 20 bytes

struct PID_STATE {
  uint8_t version;          // 1
  uint8_t flags;            // 1
  uint16_t masterPidBlend;  // 2
  float masterPidKp;        // 4
  float masterPidKi;        // 4
  float masterPidKd;        // 4
  float masterPidKs;        // 4
};                          // = 20 bytes

struct PCCMD {
  uint8_t header;    // 1
  uint8_t len;       // 1
  uint8_t cmd;       // 1
  uint8_t reserved;  // 1
  int32_t data[4];   // 16
};                   // = 20 bytes

struct PCCMD_SH {
  uint8_t header;
  uint8_t len;
  uint8_t cmd;
  uint8_t reserved;
  uint16_t data[4];
  uint16_t data2[4];
};

#pragma pack(pop)

// ============================================================================
// CONSTANTS
// ============================================================================
const int STATE_LEN = 20;
const int PID_STATE_LEN = 20;
const int RAW_DATA_LEN = 20;
const int NUM_AXES = 4;
const int PID_STATE_ID = 255;

// ============================================================================
// PIN CONFIGURATION (STM32F407VET6)
// ============================================================================
#define LED_PIN PA6
#define SERIAL_LED_PIN PA7
#define ALARM_PIN PA0

// Axis 0 - Front Left
#define STEP_PIN_0 PE8
#define DIR_PIN_0 PE9
#define LIMIT_PIN_0 PD0

// Axis 1 - Rear Left
#define STEP_PIN_1 PE10
#define DIR_PIN_1 PE11
#define LIMIT_PIN_1 PD1

// Axis 2 - Rear Right
#define STEP_PIN_2 PE12
#define DIR_PIN_2 PE13
#define LIMIT_PIN_2 PD4

// Axis 3 - Front Right
#define STEP_PIN_3 PE14
#define DIR_PIN_3 PE15
#define LIMIT_PIN_3 PD5

// ============================================================================
// BALLSCREW CONFIGURATION
// ============================================================================
#define SFU1610
#ifdef SFU1610
const float MM_PER_REV = 10.0f;
const float MAX_REVOLUTIONS = 9.0f;
const int32_t STEPS_PER_REVOLUTIONS = 1000;
const int32_t SAFE_DIST_IN_STEPS = 250;
#define MAX_SPEED_MM_SEC 400
#else
const float MM_PER_REV = 5.0f;
const float MAX_REVOLUTIONS = 17.5f;
const int32_t STEPS_PER_REVOLUTIONS = 1000;
const int32_t SAFE_DIST_IN_STEPS = 500;
#define MAX_SPEED_MM_SEC 120
#endif

const int32_t RANGE = (int32_t)(MAX_REVOLUTIONS * STEPS_PER_REVOLUTIONS);
const int32_t MIN_POS = SAFE_DIST_IN_STEPS;
const int32_t MAX_POS = RANGE - SAFE_DIST_IN_STEPS;
const uint8_t HOME_DIRECTION = HIGH;

#define MIN_PULSE_DELAY 5
#define MIN_REVERSE_DELAY 6
#define MIN_SPEED_MM_SEC 10
#define SLOW_SPEED_MM_SEC 10
#define DEFAULT_SPEED_MM_SEC 90

#define MMPERSEC2DELAY(mmps) (1000000UL / (uint32_t)((float)STEPS_PER_REVOLUTIONS * (float)mmps / MM_PER_REV))
#define MAX(a, b) ((a) > (b) ? (a) : (b))
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#define ABS(x) ((x) < 0 ? -(x) : (x))
#define CLAMP(x, a, b) ((x) < (a) ? (a) : ((x) > (b) ? (b) : (x)))

const int HOMEING_PULSE_DELAY = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(MIN_SPEED_MM_SEC) - MIN_PULSE_DELAY);
const int FAST_PULSE_DELAY = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(DEFAULT_SPEED_MM_SEC) - MIN_PULSE_DELAY);
const int SLOW_PULSE_DELAY = MAX(FAST_PULSE_DELAY * 2, (int)MMPERSEC2DELAY(SLOW_SPEED_MM_SEC) - MIN_PULSE_DELAY);
#define STEPS_CONTROL_DIST (STEPS_PER_REVOLUTIONS / 4)

// ============================================================================
// PID CONTROLLER STRUCTURE
// ============================================================================
struct PIDController {
  float Kp = 1.5f;
  float Ki = 0.0f;  // MUST stay 0 for servo drives with internal PID
  float Kd = 0.02f;
  float Ks = 0.50f;  // Derivative smoothing ratio (0..1)
  float integralLimit = 400.0f;
  float maxFreq = 40000.0f;
  float minFreq = 100.0f;
  float integral = 0.0f;
  float prevError = 0.0f;
  float derivativeFilter = 0.0f;
  uint32_t lastUpdateTime = 0;
};

// ============================================================================
// AXIS DATA STRUCTURE (with Fast GPIO fields)
// ============================================================================
struct AxisData {
  // Standard pin definitions
  uint32_t stepPin;
  uint32_t dirPin;
  uint32_t limitPin;

  // Fast GPIO fields (pre-calculated for ISR speed)
  GPIO_Regs* stepPort;  // Pointer to GPIO port register struct
  uint8_t stepPinBit;   // Pin bit number (0-15)
  GPIO_Regs* dirPort;
  uint8_t dirPinBit;
  GPIO_Regs* limitPort;
  uint8_t limitPinBit;

  // State
  STATE state;
  volatile int32_t currentPos;
  volatile int32_t targetPos;
  volatile int32_t homeTargetPos;
  volatile uint8_t currentDir;
  volatile bool bHomed;
  volatile uint8_t limitSwitchState;
  volatile bool steppingEnabled;
  volatile bool stepPinState;
  volatile uint32_t currentFrequency;
  volatile uint32_t isrCount;
  volatile uint32_t stepCount;
  int fastPulseDelay;
  int slowPulseDelay;
  PIDController pid;
  bool pidControlEnabled;
  float pidBlend;
  float limitedFrequencyHz;
  uint32_t frequencyLimiterLastMs;
  int32_t maxAccelSteps;
  HOME_SUBSTATE homeSubState;
  volatile bool isHoming;
};

// ============================================================================
// GLOBAL VARIABLES
// ============================================================================
USBSerial SerialMy;
AxisData axes[NUM_AXES];
PID_STATE pid_state;
FlashConfig flashConfig;
volatile bool estopLatched = false;
PCCMD pccmd;
PCCMD_SH& pccmd_sh = (PCCMD_SH&)pccmd;
uint8_t buf[32];
int offset = 0;
volatile bool _bDataPresent = false;
HardwareTimer* Timers[NUM_AXES] = { &Timer2, &Timer3, &Timer4, &Timer5 };
volatile uint32_t cmdCount = 0;
volatile uint32_t moveCount = 0;
uint32_t lastStatsMs = 0;
uint32_t loopCount = 0;
static bool configLoaded = false;
static bool configDirty = false;

// Pre-calculated LED state for fast toggle
static volatile uint8_t ledOdrMask = 0;  // Shadow register for LED state

// ============================================================================
// HELPER FUNCTIONS
// ============================================================================
static inline uint32_t mmPerSecToFreq(uint32_t mmPerSec) {
  return (uint32_t)((float)STEPS_PER_REVOLUTIONS * (float)mmPerSec / MM_PER_REV);
}

static inline uint32_t delayToFreq(uint32_t delayUs) {
  if (delayUs < MIN_PULSE_DELAY) delayUs = MIN_PULSE_DELAY;
  return 1000000 / (delayUs + MIN_PULSE_DELAY);
}

static inline void resetPID(AxisData* axis) {
  axis->pid.integral = 0.0f;
  axis->pid.prevError = (float)(axis->targetPos - axis->currentPos);
  axis->pid.derivativeFilter = 0.0f;
  axis->pid.lastUpdateTime = millis();
  axis->limitedFrequencyHz = 0.0f;
  axis->frequencyLimiterLastMs = millis();
}

static inline uint32_t applyAccelLimit(AxisData* axis, uint32_t desiredFreqHz) {
  uint32_t nowMs = millis();
  float dt = (nowMs - axis->frequencyLimiterLastMs) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;
  axis->frequencyLimiterLastMs = nowMs;

  float maxDelta = (float)axis->maxAccelSteps * dt;
  float delta = (float)desiredFreqHz - axis->limitedFrequencyHz;

  if (ABS(delta) > maxDelta) {
    axis->limitedFrequencyHz += (delta > 0.0f) ? maxDelta : -maxDelta;
  } else {
    axis->limitedFrequencyHz = (float)desiredFreqHz;
  }

  if (axis->limitedFrequencyHz < 0.0f) axis->limitedFrequencyHz = 0.0f;
  else if (axis->limitedFrequencyHz > 40000.0f) axis->limitedFrequencyHz = 40000.0f;

  return (uint32_t)(axis->limitedFrequencyHz + 0.5f);
}

// ============================================================================
// PID COMPUTATION
// ============================================================================
static inline float computePID(AxisData* axis, uint32_t currentTime) {
  float dt = (currentTime - axis->pid.lastUpdateTime) / 1000.0f;
  if (dt <= 0.0f || dt > 0.5f) dt = 0.001f;
  axis->pid.lastUpdateTime = currentTime;

  float error = (float)(axis->targetPos - axis->currentPos);
  float proportional = axis->pid.Kp * error;

  // Anti-windup: only integrate when close to target
  if (fabs(error) < 800.0f) {
    axis->pid.integral += error * dt;
    axis->pid.integral = CLAMP(axis->pid.integral, -axis->pid.integralLimit, axis->pid.integralLimit);
  } else {
    axis->pid.integral *= 0.9f;  // Decay integral when far from target
  }

  float integral = axis->pid.Ki * axis->pid.integral;

  // Derivative with filtering
  float derivative = (error - axis->pid.prevError) / dt;
  axis->pid.derivativeFilter = axis->pid.Ks * axis->pid.derivativeFilter + (1.0f - axis->pid.Ks) * derivative;
  float derivativeTerm = axis->pid.Kd * axis->pid.derivativeFilter;

  axis->pid.prevError = error;

  float output = proportional + integral + derivativeTerm;
  output *= axis->pidBlend;

  float freq = fabs(output);

  // Dead zone
  if (freq < axis->pid.minFreq && fabs(error) < 2.0f) {
    return 0.0f;
  }

  // Clamp frequency
  if (freq < axis->pid.minFreq) freq = axis->pid.minFreq;
  if (freq > axis->pid.maxFreq) freq = axis->pid.maxFreq;

  return freq;
}

// ============================================================================
// TIMER ISR - Parallel execution for all axes (with Fast GPIO)
// ============================================================================
static inline bool checkHomingTarget(AxisData* axis) {
  if (!axis->isHoming) return false;
  if (axis->currentDir == HIGH) {
    return axis->currentPos >= axis->homeTargetPos;
  } else {
    return axis->currentPos <= axis->homeTargetPos;
  }
}

// Main ISR - uses Fast GPIO for minimum jitter
static inline void processTimerISR(AxisData* axis) {
  axis->isrCount++;

  if (!axis->steppingEnabled) {
    PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO
    return;
  }

  // Check target position
  if (axis->isHoming) {
    if (checkHomingTarget(axis)) {
      axis->steppingEnabled = false;
      PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO
      return;
    }
  } else {
    if (axis->currentPos == axis->targetPos) {
      axis->steppingEnabled = false;
      PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO
      return;
    }
  }

  // Check limit switch - stop if moving towards it
  if (axis->limitSwitchState == HIGH && axis->currentDir == HOME_DIRECTION) {
    axis->steppingEnabled = false;
    PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO
    return;
  }

  // Generate STEP pulse (toggle on each ISR call)
  // Using Fast GPIO macros for maximum speed
  if (!axis->stepPinState) {
    PIN_SET(axis->stepPort, axis->stepPinBit);  // Fast GPIO - set HIGH
    axis->stepPinState = true;
  } else {
    PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO - set LOW
    axis->stepPinState = false;
    axis->currentPos += (axis->currentDir == HIGH) ? 1 : -1;
    axis->stepCount++;
  }
}

void timerISR_0() {
  processTimerISR(&axes[0]);
}
void timerISR_1() {
  processTimerISR(&axes[1]);
}
void timerISR_2() {
  processTimerISR(&axes[2]);
}
void timerISR_3() {
  processTimerISR(&axes[3]);
}

// ============================================================================
// STEPPING CONTROL (with Fast GPIO)
// ============================================================================
void initStepTimer(int axisIndex) {
  if (Timers[axisIndex] == nullptr) return;

  Timers[axisIndex]->init();
  Timers[axisIndex]->pause();
  Timers[axisIndex]->setPrescaleFactor(1);   // 168MHz for F407
  Timers[axisIndex]->setOverflow(4200 - 1);  // Initial ~1282Hz

  Timers[axisIndex]->setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);

  voidFuncPtr isrFunc = timerISR_0;
  if (axisIndex == 1) isrFunc = timerISR_1;
  else if (axisIndex == 2) isrFunc = timerISR_2;
  else if (axisIndex == 3) isrFunc = timerISR_3;

  Timers[axisIndex]->attachInterrupt(TIMER_CH1, isrFunc);
  Timers[axisIndex]->refresh();
  Timers[axisIndex]->resume();
}

static inline void setStepFrequency(int axisIndex, uint32_t freqHz) {
  if (freqHz < 100) freqHz = 100;
  if (freqHz > 40000) freqHz = 40000;
  if (Timers[axisIndex] == nullptr) return;

  axes[axisIndex].currentFrequency = freqHz;

  // For 168MHz F407: overflow = 168000000 / (freqHz * 2)
  uint32_t timerFreq = freqHz * 2;
  uint32_t overflow = 168000000UL / timerFreq;

  // 16-bit timer limit
  if (overflow < 100) overflow = 100;
  if (overflow > 65535) overflow = 65535;

  Timers[axisIndex]->setOverflow(overflow);
  Timers[axisIndex]->refresh();
}

static inline void startStepping(AxisData* axis, uint8_t dir) {
  axis->steppingEnabled = false;
  PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO
  axis->stepPinState = false;

  if (axis->currentDir != dir) {
    delayMicroseconds(MIN_REVERSE_DELAY);

    // Fast GPIO for direction pin
    if (dir == HIGH) {
      PIN_SET(axis->dirPort, axis->dirPinBit);
    } else {
      PIN_CLR(axis->dirPort, axis->dirPinBit);
    }

    axis->currentDir = dir;
    delayMicroseconds(MIN_REVERSE_DELAY);
  }

  axis->stepPinState = false;
  axis->steppingEnabled = true;
}

static inline void stopStepping(AxisData* axis) {
  axis->steppingEnabled = false;
  PIN_CLR(axis->stepPort, axis->stepPinBit);  // Fast GPIO
  axis->stepPinState = false;
}

static inline void updateLimitSwitch(AxisData* axis) {
  // Fast GPIO read for limit switch
  axis->limitSwitchState = PIN_READ(axis->limitPort, axis->limitPinBit);
}

// ============================================================================
// AXIS INITIALIZATION (with Fast GPIO setup)
// ============================================================================
void initAxis(int index, uint32_t stepPin, uint32_t dirPin, uint32_t limitPin) {
  AxisData* axis = &axes[index];

  // Store standard pin numbers
  axis->stepPin = stepPin;
  axis->dirPin = dirPin;
  axis->limitPin = limitPin;

  // Calculate Fast GPIO port and bit (pre-calc for ISR speed)
  axis->stepPort = getGpioPort(stepPin);
  axis->stepPinBit = getGpioPinBit(stepPin);
  axis->dirPort = getGpioPort(dirPin);
  axis->dirPinBit = getGpioPinBit(dirPin);
  axis->limitPort = getGpioPort(limitPin);
  axis->limitPinBit = getGpioPinBit(limitPin);

  // Configure pins (still use standard functions for setup)
  pinMode(stepPin, OUTPUT);
  pinMode(dirPin, OUTPUT);
  pinMode(limitPin, INPUT_PULLDOWN);

  digitalWrite(stepPin, LOW);
  digitalWrite(dirPin, LOW);

  // Initialize state
  axis->state.mode = (uint8_t)MODE::CONNECTED;
  axis->state.flags = 0;
  axis->state.speedMMperSEC = (uint16_t)DEFAULT_SPEED_MM_SEC;
  axis->state.min = MIN_POS;
  axis->state.max = MAX_POS;

  axis->currentPos = 0;
  axis->targetPos = (MIN_POS + MAX_POS) / 2;
  axis->homeTargetPos = 0;
  axis->currentDir = LOW;
  axis->bHomed = false;
  axis->limitSwitchState = PIN_READ(axis->limitPort, axis->limitPinBit);  // Fast GPIO read
  axis->steppingEnabled = false;
  axis->stepPinState = false;
  axis->isrCount = 0;
  axis->stepCount = 0;
  axis->currentFrequency = 0;

  axis->fastPulseDelay = FAST_PULSE_DELAY;
  axis->slowPulseDelay = SLOW_PULSE_DELAY;
  axis->pid.maxFreq = (float)mmPerSecToFreq(MAX_SPEED_MM_SEC);
  axis->pidControlEnabled = false;
  axis->pidBlend = 1.0f;
  axis->maxAccelSteps = (int32_t)(25000.0f * STEPS_PER_REVOLUTIONS / MM_PER_REV);
  axis->homeSubState = HOME_IDLE;
  axis->isHoming = false;

  resetPID(axis);
  initStepTimer(index);
}

// ============================================================================
// NON-BLOCKING HOMING
// ============================================================================
void startHoming(AxisData* axis, int axisIndex) {
  axis->homeSubState = HOME_SEEKING_LIMIT;
  axis->bHomed = false;
  axis->currentPos = 0;
  axis->isHoming = true;
  axis->stepCount = 0;
  axis->isrCount = 0;
  axis->state.mode = (uint8_t)MODE::HOMEING;

  updateLimitSwitch(axis);

  uint32_t freq = delayToFreq(HOMEING_PULSE_DELAY);

  if (axis->limitSwitchState == HIGH) {
    // Limit already pressed - start with retract
    axis->homeSubState = HOME_RETRACTING;
    axis->homeTargetPos = -SAFE_DIST_IN_STEPS;
    axis->targetPos = axis->homeTargetPos;
    setStepFrequency(axisIndex, freq);
    startStepping(axis, !HOME_DIRECTION);
  } else {
    // Go find limit switch
    axis->homeTargetPos = RANGE * 2;
    axis->targetPos = axis->homeTargetPos;
    setStepFrequency(axisIndex, freq);
    startStepping(axis, HOME_DIRECTION);
  }
}

void processHoming(AxisData* axis, int axisIndex) {
  updateLimitSwitch(axis);

  switch (axis->homeSubState) {
    case HOME_SEEKING_LIMIT:
      if (axis->limitSwitchState == HIGH) {
        stopStepping(axis);
        axis->homeSubState = HOME_RETRACTING;
        axis->homeTargetPos = axis->currentPos - (HOME_DIRECTION == HIGH ? SAFE_DIST_IN_STEPS : -SAFE_DIST_IN_STEPS);
        axis->targetPos = axis->homeTargetPos;
        setStepFrequency(axisIndex, delayToFreq(HOMEING_PULSE_DELAY));
        startStepping(axis, !HOME_DIRECTION);
      } else if (ABS(axis->currentPos) >= RANGE * 1.5) {
        stopStepping(axis);
        axis->state.mode = (uint8_t)MODE::ALARM;
        axis->homeSubState = HOME_IDLE;
        axis->isHoming = false;
      } else if (!axis->steppingEnabled) {
        setStepFrequency(axisIndex, delayToFreq(HOMEING_PULSE_DELAY));
        startStepping(axis, HOME_DIRECTION);
      }
      break;

    case HOME_RETRACTING:
      {
        bool done;
        if (HOME_DIRECTION == HIGH) {
          done = (axis->currentDir == LOW) && (axis->currentPos <= axis->homeTargetPos);
        } else {
          done = (axis->currentDir == HIGH) && (axis->currentPos >= axis->homeTargetPos);
        }

        if (done) {
          stopStepping(axis);
          axis->homeSubState = HOME_MOVING_CENTER;
          axis->currentPos = MAX_POS;
          axis->targetPos = (MIN_POS + MAX_POS) / 2;
          axis->homeTargetPos = axis->targetPos;
          setStepFrequency(axisIndex, delayToFreq(HOMEING_PULSE_DELAY));
          startStepping(axis, (axis->targetPos > axis->currentPos) ? HIGH : LOW);
        } else if (!axis->steppingEnabled) {
          setStepFrequency(axisIndex, delayToFreq(HOMEING_PULSE_DELAY));
          startStepping(axis, !HOME_DIRECTION);
        }
      }
      break;

    case HOME_MOVING_CENTER:
      if (ABS(axis->currentPos - axis->homeTargetPos) < 5) {
        stopStepping(axis);
        axis->homeSubState = HOME_DONE;
        axis->isHoming = false;
        axis->state.mode = (uint8_t)MODE::READY;
        axis->bHomed = true;
        axis->state.flags |= 2;
        resetPID(axis);
      } else if (!axis->steppingEnabled) {
        setStepFrequency(axisIndex, delayToFreq(HOMEING_PULSE_DELAY));
        startStepping(axis, (axis->homeTargetPos > axis->currentPos) ? HIGH : LOW);
      }
      break;

    default:
      break;
  }
}

// ============================================================================
// READY MOTION WITH PID SUPPORT
// ============================================================================
static inline void processReadyMotion(AxisData* axis, int axisIndex) {
  int32_t error = axis->targetPos - axis->currentPos;
  int32_t absError = ABS(error);

  if (absError > 2) {
    uint8_t dir = (error > 0) ? HIGH : LOW;
    uint32_t desiredFreqHz = 0;

    if (axis->pidControlEnabled) {
      // Use PID controller
      desiredFreqHz = (uint32_t)computePID(axis, millis());
    } else {
      // Legacy motion profiling
      if (absError > STEPS_CONTROL_DIST) {
        desiredFreqHz = delayToFreq(axis->fastPulseDelay);
      } else if (absError > STEPS_CONTROL_DIST / 4) {
        desiredFreqHz = delayToFreq(axis->slowPulseDelay) + (delayToFreq(axis->fastPulseDelay) - delayToFreq(axis->slowPulseDelay)) * (absError - STEPS_CONTROL_DIST / 4) * 4 / STEPS_CONTROL_DIST;
      } else {
        desiredFreqHz = delayToFreq(axis->slowPulseDelay) * absError * 4 / STEPS_CONTROL_DIST;
      }
    }

    uint32_t freqHz = applyAccelLimit(axis, desiredFreqHz);

    if (freqHz > 100) {
      setStepFrequency(axisIndex, freqHz);
      if (!axis->steppingEnabled) {
        if (axis->pidControlEnabled) resetPID(axis);
        startStepping(axis, dir);
      } else if (axis->currentDir != dir) {
        stopStepping(axis);
        delayMicroseconds(100);
        if (axis->pidControlEnabled) resetPID(axis);
        startStepping(axis, dir);
      }
    }
  } else {
    stopStepping(axis);
    if (axis->pidControlEnabled) {
      axis->pid.integral *= 0.95f;  // Decay integral near target
    }
  }
}

// ============================================================================
// SERIAL COMMUNICATION (with Fast GPIO for LED)
// ============================================================================
void readSerialData() {
  int avail = SerialMy.available();
  if (avail == 0) return;

  PIN_CLR(SERIAL_LED_PORT, SERIAL_LED_BIT);  // Fast GPIO - LED ON

  while (avail-- > 0) {
    int byte = SerialMy.read();

    if (offset > 0) {
      buf[offset++] = byte;
      if (offset == RAW_DATA_LEN) {
        memcpy(&pccmd, buf, RAW_DATA_LEN);
        _bDataPresent = true;
        offset = 0;
        break;
      }
    } else {
      if (byte == 0) {
        buf[offset++] = byte;
      }
    }
  }

  PIN_SET(SERIAL_LED_PORT, SERIAL_LED_BIT);  // Fast GPIO - LED OFF
}

// ============================================================================
// COMMAND PROCESSING
// ============================================================================
void ProcessCommand() {
  if (!_bDataPresent) return;

  cmdCount++;
  _bDataPresent = false;

  switch (pccmd.cmd) {
    case COMMAND::CMD_HOME:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (pccmd.data[i] == 1) startHoming(&axes[i], i);
        }
      }
      break;

    case COMMAND::CMD_ENABLE:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (pccmd.data[i] == 1) {
            axes[i].state.mode = axes[i].bHomed ? (uint8_t)MODE::READY : (uint8_t)MODE::CONNECTED;
            resetPID(&axes[i]);
          }
        }
      }
      break;

    case COMMAND::CMD_DISABLE:
      for (int i = 0; i < NUM_AXES; i++) {
        if (pccmd.data[i] == 1) {
          axes[i].state.mode = (uint8_t)MODE::DISABLED;
          axes[i].isHoming = false;
          stopStepping(&axes[i]);
        }
      }
      break;

    case COMMAND::SET_ALARM:
      for (int i = 0; i < NUM_AXES; i++) {
        if (pccmd.data[i] == 1) {
          axes[i].bHomed = false;
          axes[i].state.mode = (uint8_t)MODE::ALARM;
          axes[i].state.flags &= ~2;
          axes[i].isHoming = false;
          stopStepping(&axes[i]);
        }
      }
      break;

    case COMMAND::CMD_CLEAR_ALARM:
      if (digitalRead(ALARM_PIN) != HIGH) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (pccmd.data[i] == 1) {
            axes[i].state.mode = axes[i].bHomed ? (uint8_t)MODE::READY : (uint8_t)MODE::CONNECTED;
          }
        }
        estopLatched = false;
      }
      break;

    case COMMAND::CMD_PARK:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (axes[i].bHomed) {
            axes[i].state.mode = (uint8_t)MODE::PARKING;
            axes[i].targetPos = MIN_POS;
            setStepFrequency(i, delayToFreq(HOMEING_PULSE_DELAY));
            startStepping(&axes[i], (axes[i].targetPos > axes[i].currentPos) ? HIGH : LOW);
          }
        }
      }
      break;

    case COMMAND::CMD_MOVE_SH:
      if (!estopLatched) {
        moveCount++;
        for (int i = 0; i < NUM_AXES; i++) {
          if (axes[i].bHomed && axes[i].state.mode == (uint8_t)MODE::READY) {
            uint16_t val = (pccmd_sh.data[i] >> 8) | (pccmd_sh.data[i] << 8);
            axes[i].targetPos = MIN_POS + (int32_t)val * (MAX_POS - MIN_POS) / 65535;
          }
        }
      }
      break;

    case COMMAND::CMD_MOVE:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          if (axes[i].bHomed && axes[i].state.mode == (uint8_t)MODE::READY) {
            int32_t v = pccmd.data[i];
            if (v < MIN_POS) v = MIN_POS;
            else if (v > MAX_POS) v = MAX_POS;
            axes[i].targetPos = v;
          }
        }
      }
      break;

    case COMMAND::CMD_SET_SPEED:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          int speed = pccmd.data[i];
          if (speed < 10) speed = 10;
          else if (speed > MAX_SPEED_MM_SEC) speed = MAX_SPEED_MM_SEC;
          axes[i].state.speedMMperSEC = (uint16_t)speed;
          axes[i].fastPulseDelay = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(speed) - MIN_PULSE_DELAY);
          axes[i].pid.maxFreq = (float)mmPerSecToFreq(speed);
        }
        configDirty = true;
      }
      break;

    case COMMAND::CMD_SET_SLOW_SPEED:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          int speed = pccmd.data[i];
          if (speed < 10) speed = 10;
          axes[i].slowPulseDelay = MAX(MIN_PULSE_DELAY, (int)MMPERSEC2DELAY(speed) - MIN_PULSE_DELAY);
        }
      }
      break;

    case COMMAND::CMD_SET_ACCEL:
      if (!estopLatched) {
        for (int i = 0; i < NUM_AXES; i++) {
          int32_t accel = pccmd.data[i];
          axes[i].maxAccelSteps = (int32_t)((float)accel * STEPS_PER_REVOLUTIONS / MM_PER_REV);
        }
        configDirty = true;
      }
      break;

    // PID Commands
    case COMMAND::CMD_SET_PID_KP:
    case COMMAND::CMD_SET_PID_KI:
    case COMMAND::CMD_SET_PID_KD:
    case COMMAND::CMD_SET_PID_KS:
    case COMMAND::CMD_SET_PID_ENABLE:
    case COMMAND::CMD_SET_PID_BLEND:
      if (!estopLatched) {
        float value = (float)pccmd.data[0];
        for (int i = 0; i < NUM_AXES; i++) {
          switch (pccmd.cmd) {
            case COMMAND::CMD_SET_PID_KP:
              axes[i].pid.Kp = CLAMP(value / 10.0f, 0.0f, 200.0f);
              break;
            case COMMAND::CMD_SET_PID_KI:
              axes[i].pid.Ki = CLAMP(value / 10.0f, 0.0f, 50.0f);
              break;
            case COMMAND::CMD_SET_PID_KD:
              axes[i].pid.Kd = CLAMP(value / 100.0f, 0.0f, 50.0f);
              break;
            case COMMAND::CMD_SET_PID_KS:
              axes[i].pid.Ks = CLAMP(value / 100.0f, 0.0f, 1.0f);
              break;
            case COMMAND::CMD_SET_PID_ENABLE:
              axes[i].pidControlEnabled = (value != 0);
              break;
            case COMMAND::CMD_SET_PID_BLEND:
              axes[i].pidBlend = CLAMP(value / 100.0f, 0.0f, 1.0f);
              break;
          }
        }
        // Update pid_state for Plugin.cs
        pid_state.masterPidKp = axes[0].pid.Kp;
        pid_state.masterPidKi = axes[0].pid.Ki;
        pid_state.masterPidKd = axes[0].pid.Kd;
        pid_state.masterPidKs = axes[0].pid.Ks;
        pid_state.masterPidBlend = (uint16_t)(axes[0].pidBlend * 100);

        // Update PID flags
        if (pccmd.cmd == COMMAND::CMD_SET_PID_ENABLE) {
          if (value != 0) pid_state.flags |= PID_ENABLED;
          else pid_state.flags &= ~PID_ENABLED;
        }
        configDirty = true;
      }
      break;

    case COMMAND::CMD_GET_STATE:
      for (int i = 0; i < NUM_AXES; i++) {
        axes[i].state.currentpos = axes[i].currentPos;
        axes[i].state.targetpos = axes[i].targetPos;
        axes[i].state.flags = 0;
        if (axes[i].limitSwitchState == HIGH) axes[i].state.flags |= 1;
        if (axes[i].bHomed) axes[i].state.flags |= 2;

        SerialMy.write(10 + i);
        SerialMy.write(STATE_LEN);
        SerialMy.write((uint8_t*)&axes[i].state, STATE_LEN);
      }
      SerialMy.flush();
      break;

    case COMMAND::CMD_GET_PID_STATE:
      SerialMy.write(PID_STATE_ID);
      SerialMy.write(PID_STATE_LEN);
      SerialMy.write((uint8_t*)&pid_state, PID_STATE_LEN);
      SerialMy.flush();
      break;

    case COMMAND::CMD_STORE_PID:
      if (!estopLatched) {
        flashConfig.pidKp = axes[0].pid.Kp;
        flashConfig.pidKi = axes[0].pid.Ki;
        flashConfig.pidKd = axes[0].pid.Kd;
        flashConfig.pidKs = axes[0].pid.Ks;
        flashConfig.pidBlend = (uint16_t)(axes[0].pidBlend * 100.0f);
        flashConfig.pidFlags = (uint16_t)pid_state.flags;
        flashConfig.defaultSpeed = axes[0].state.speedMMperSEC;
        flashConfig.acceleration = (uint32_t)(axes[0].maxAccelSteps * MM_PER_REV / STEPS_PER_REVOLUTIONS);
        if (SpiFlashStorage::saveConfig(flashConfig)) {
          configDirty = false;
#ifdef DEBUG
          SerialMy.println("Flash: Saved");
#endif
        }
      }
      break;

    case COMMAND::CMD_RESTORE_PID:
      if (!estopLatched) {
        SpiFlashStorage::loadConfig(flashConfig);
        for (int i = 0; i < NUM_AXES; i++) {
          axes[i].pid.Kp = flashConfig.pidKp;
          axes[i].pid.Ki = flashConfig.pidKi;
          axes[i].pid.Kd = flashConfig.pidKd;
          axes[i].pid.Ks = flashConfig.pidKs;
          axes[i].pidBlend = flashConfig.pidBlend / 100.0f;
          axes[i].pidControlEnabled = (flashConfig.pidFlags & PID_ENABLED) != 0;
        }
        pid_state.masterPidKp = flashConfig.pidKp;
        pid_state.masterPidKi = flashConfig.pidKi;
        pid_state.masterPidKd = flashConfig.pidKd;
        pid_state.masterPidKs = flashConfig.pidKs;
        pid_state.masterPidBlend = flashConfig.pidBlend;
        pid_state.flags = (uint8_t)flashConfig.pidFlags;
        configDirty = false;
      }
      break;

    default:
      break;
  }
}

// ============================================================================
// LED STATUS INDICATOR (with Fast GPIO)
// ============================================================================
void updateLedStatus() {
  static uint32_t lastToggle = 0;
  static bool ledState = false;
  uint32_t now = millis();

  bool anyHoming = false;
  bool allReady = true;

  for (int i = 0; i < NUM_AXES; i++) {
    if (axes[i].homeSubState != HOME_IDLE && axes[i].homeSubState != HOME_DONE) anyHoming = true;
    if (axes[i].state.mode != (uint8_t)MODE::READY || !axes[i].bHomed) allReady = false;
  }

  uint16_t blinkPeriod = 0;

  if (estopLatched) blinkPeriod = 100;
  else if (anyHoming) blinkPeriod = 500;
  else if (allReady) {
    PIN_CLR(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED ON (steady)
    return;
  } else blinkPeriod = 1000;

  if (now - lastToggle >= blinkPeriod) {
    lastToggle = now;
    ledState = !ledState;

    if (ledState) {
      PIN_CLR(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED ON
    } else {
      PIN_SET(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED OFF
    }
  }
}

// ============================================================================
// MAIN AXIS PROCESSING - Parallel execution
// ============================================================================
static inline void ProcessAxisMotion(int index) {
  AxisData* axis = &axes[index];
  updateLimitSwitch(axis);

  switch (axis->state.mode) {
    case (uint8_t)MODE::HOMEING:
      processHoming(axis, index);
      break;
    case (uint8_t)MODE::PARKING:
      if (ABS(axis->currentPos - axis->targetPos) < 5) {
        stopStepping(axis);
        axis->state.mode = (uint8_t)MODE::READY;
        resetPID(axis);
      } else if (!axis->steppingEnabled) {
        setStepFrequency(index, delayToFreq(HOMEING_PULSE_DELAY));
        startStepping(axis, (axis->targetPos > axis->currentPos) ? HIGH : LOW);
      }
      break;
    case (uint8_t)MODE::READY:
      processReadyMotion(axis, index);
      break;
    case (uint8_t)MODE::ALARM:
    case (uint8_t)MODE::DISABLED:
      axis->isHoming = false;
      stopStepping(axis);
      break;
    default:
      break;
  }
}

// ============================================================================
// DEBUG OUTPUT
// ============================================================================
void printDebugStatus() {
  SerialMy.println("\r\n========== DEBUG STATUS ==========");
  for (int i = 0; i < NUM_AXES; i++) {
    SerialMy.print("Axis ");
    SerialMy.print(i);
    SerialMy.print(": mode=");
    SerialMy.print((int)axes[i].state.mode);
    SerialMy.print(" sub=");
    SerialMy.print((int)axes[i].homeSubState);
    SerialMy.print(" pos=");
    SerialMy.print((int32_t)axes[i].currentPos);
    SerialMy.print(" tgt=");
    SerialMy.print((int32_t)axes[i].targetPos);
    SerialMy.println("");
    SerialMy.print("       lim=");
    SerialMy.print((int)axes[i].limitSwitchState);
    SerialMy.print(" en=");
    SerialMy.print(axes[i].steppingEnabled ? "Y " : "N ");
    SerialMy.print(" dir=");
    SerialMy.print((int)axes[i].currentDir);
    SerialMy.println(" ");
    SerialMy.print("       isrCount=");
    SerialMy.print((uint32_t)axes[i].isrCount);
    SerialMy.print(" stepCount=");
    SerialMy.print((uint32_t)axes[i].stepCount);
    SerialMy.print(" freq=");
    SerialMy.print((uint32_t)axes[i].currentFrequency);
    SerialMy.println(" Hz ");
  }
  SerialMy.print("cmdCount=");
  SerialMy.print((uint32_t)cmdCount);
  SerialMy.print(" loopCount=");
  SerialMy.println(loopCount);
  SerialMy.println("==================================");
}

// ============================================================================
// SETUP
// ============================================================================
void setup() {
  // Initialize LED pins (Fast GPIO compatible)
  pinMode(LED_PIN, OUTPUT);
  pinMode(SERIAL_LED_PIN, OUTPUT);
  pinMode(ALARM_PIN, INPUT_PULLDOWN);

  // Initial state - LEDs OFF
  PIN_SET(LED_PORT, LED_PIN_BIT);            // Fast GPIO - LED OFF
  PIN_SET(SERIAL_LED_PORT, SERIAL_LED_BIT);  // Fast GPIO - LED OFF

  // Startup blink (5 fast blinks)
  for (int i = 0; i < 5; i++) {
    PIN_CLR(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED ON
    delay(50);
    PIN_SET(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED OFF
    delay(50);
  }

  // Check ALARM pin (E-STOP)
  estopLatched = (PIN_READ(ALARM_PORT, ALARM_PIN_BIT) == 1);  // Fast GPIO read

  // Initialize PID state defaults
  pid_state.version = 1;
  pid_state.flags = PID_NONE;
  pid_state.masterPidBlend = 100;
  pid_state.masterPidKp = 15.0f;
  pid_state.masterPidKi = 0.0f;
  pid_state.masterPidKd = 0.02f;
  pid_state.masterPidKs = 0.30f;

  SerialMy.begin(115200);
  while (!SerialMy)
    ;
  delay(1000);

  // === INITIALIZE SPI FLASH ===
  if (SpiFlashStorage::begin()) {
    // === LOAD CONFIGURATION ===
    if (SpiFlashStorage::loadConfig(flashConfig)) {
      // Apply loaded settings
      for (int i = 0; i < NUM_AXES; i++) {
        axes[i].pid.Kp = flashConfig.pidKp;
        axes[i].pid.Ki = flashConfig.pidKi;
        axes[i].pid.Kd = flashConfig.pidKd;
        axes[i].pid.Ks = flashConfig.pidKs;
        axes[i].pidBlend = flashConfig.pidBlend / 100.0f;
        axes[i].state.speedMMperSEC = flashConfig.defaultSpeed;
        axes[i].fastPulseDelay = MAX(MIN_PULSE_DELAY,
                                     (int)MMPERSEC2DELAY(flashConfig.defaultSpeed) - MIN_PULSE_DELAY);
        axes[i].pid.maxFreq = (float)mmPerSecToFreq(flashConfig.defaultSpeed);
        axes[i].maxAccelSteps = (int32_t)((float)flashConfig.acceleration * STEPS_PER_REVOLUTIONS / MM_PER_REV);
        axes[i].pidControlEnabled = (flashConfig.pidFlags & PID_ENABLED) != 0;
      }
      // Sync pid_state for Plugin.cs
      pid_state.masterPidKp = flashConfig.pidKp;
      pid_state.masterPidKi = flashConfig.pidKi;
      pid_state.masterPidKd = flashConfig.pidKd;
      pid_state.masterPidKs = flashConfig.pidKs;
      pid_state.masterPidBlend = flashConfig.pidBlend;
      pid_state.flags = (uint8_t)flashConfig.pidFlags;
      configLoaded = true;
#ifdef DEBUG
      SerialMy.println("Flash: Config loaded");
#endif
    } else {
      // Use defaults
      for (int i = 0; i < NUM_AXES; i++) {
        axes[i].pid.Kp = 1.5f;
        axes[i].pid.Ki = 0.0f;
        axes[i].pid.Kd = 0.02f;
        axes[i].pid.Ks = 0.50f;
        axes[i].pidControlEnabled = false;
      }
      pid_state.flags = PID_NONE;
#ifdef DEBUG
      SerialMy.println("Flash: Using defaults");
#endif
    }
  }

  // Initialize axes (with Fast GPIO setup)
  initAxis(0, STEP_PIN_0, DIR_PIN_0, LIMIT_PIN_0);
  initAxis(1, STEP_PIN_1, DIR_PIN_1, LIMIT_PIN_1);
  initAxis(2, STEP_PIN_2, DIR_PIN_2, LIMIT_PIN_2);
  initAxis(3, STEP_PIN_3, DIR_PIN_3, LIMIT_PIN_3);

  // Ready blink (3 slow blinks)
  for (int i = 0; i < 3; i++) {
    PIN_CLR(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED ON
    delay(200);
    PIN_SET(LED_PORT, LED_PIN_BIT);  // Fast GPIO - LED OFF
    delay(200);
  }
}

// ============================================================================
// MAIN LOOP - Non-blocking parallel execution
// ============================================================================
void loop() {
  loopCount++;

  // E-STOP check (with Fast GPIO)
  if (PIN_READ(ALARM_PORT, ALARM_PIN_BIT) && !estopLatched) {
    estopLatched = true;
    for (int i = 0; i < NUM_AXES; i++) {
      axes[i].state.mode = (uint8_t)MODE::ALARM;
      axes[i].isHoming = false;
      stopStepping(&axes[i]);
    }
  }

  // Serial communication
  readSerialData();
  ProcessCommand();

  // Process all axes in parallel (non-blocking)
  ProcessAxisMotion(0);
  ProcessAxisMotion(1);
  ProcessAxisMotion(2);
  ProcessAxisMotion(3);

  // Periodic tasks
  if ((loopCount & 0x7F) == 0) {
    updateLedStatus();

    // Auto-save configuration every 10 seconds if changed
    static uint32_t lastAutoSave = 0;
    if (configDirty && millis() - lastAutoSave > 10000) {
      flashConfig.pidKp = axes[0].pid.Kp;
      flashConfig.pidKi = axes[0].pid.Ki;
      flashConfig.pidKd = axes[0].pid.Kd;
      flashConfig.pidKs = axes[0].pid.Ks;
      flashConfig.pidBlend = (uint16_t)(axes[0].pidBlend * 100.0f);
      flashConfig.pidFlags = (uint16_t)pid_state.flags;
      flashConfig.defaultSpeed = axes[0].state.speedMMperSEC;
      flashConfig.acceleration = (uint32_t)(axes[0].maxAccelSteps * MM_PER_REV / STEPS_PER_REVOLUTIONS);
      if (SpiFlashStorage::saveConfig(flashConfig)) {
        configDirty = false;
      }
      lastAutoSave = millis();
    }

    if (millis() - lastStatsMs > 1000) {
      lastStatsMs = millis();
#ifdef DEBUG
      printDebugStatus();
#endif
      cmdCount = 0;
      moveCount = 0;
      loopCount = 0;
    }
  }
}