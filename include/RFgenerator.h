#ifndef RFgen_h
#define RFgen_h

//******************
//******************
//  Configuration options
//******************
//******************

// These defines are set in the platformIO.ini file using build_flags
// Enable this define to build the RF generator that supports fast ramping
//#define RFGEN2
// Enable this define to build the RF generator that is configurated for single ended operation
//#define SINGLEENDED
// Enable this define to build the RF generator that supports an inverted used for the trigger input buffer
//#define TRIGINVERTED
// Enable this define if the RF generator is equiped with the RF quence module for fast turn off of RF
//#define QUENCH

#include "Hardware.h"
#include "ethernet.h"

#define FILTER   0.1

#define SIGNATURE  0xAA55A5A5

extern float RFvoltageP;
extern float RFvoltageN;
extern float DriveVolt;
extern float DriveCurrent;
extern float Power;

extern bool TuneRequest;
extern bool Tuning;
extern int  MaxTuneFreq;
extern int  MinTuneFreq;

extern void (*myAnalogWriteFunction)(int);

#define  MAXPWL 10

enum ActiveLevel
{
  ACTIVE_HIGH,
  ACTIVE_LOW
};

enum TriggerOptions
{
  TRIG_OFF,
  TRIG_GATE
};

enum RampTriggerOptions
{
  RAMP_TRIG_OFF,
  RAMP_TRIG_ACTIVE,
  RAMP_TRIG_INACTIVE,
};

// This data structure is used for the piece wise linear calibration
// function.
typedef struct
{
  uint8_t   num;
  uint16_t  ADCvalue[MAXPWL];
  uint16_t  Value[MAXPWL];
} PWLcalibration;

typedef struct
{
  int16_t             Size;             // This data structures size in bytes
  char                Name[20];         // Holds the board name, "RFgen"
  int8_t              Rev;              // Holds the board revision number
  bool                Enable;
  int                 TWIadd;
  // Generator settings
  int                 Freq;             // Fragmentor operating frequency
  float               Drive;
  float               Setpoint;         // RF voltage setpoint in volts
  float               Bias;             // DC bias voltage
  int                 Relay;            // Relay state, bit 0 = Relay 1...
  bool                Mode;             // Mode = true for closed loop, false for open
  bool                Gate;             // True to enable gate mode
  float               Gain;             // Closed loop control gain
  // Limits
  float               MaxDrive;
  float               MaxPower;
  // ADC channels
  ADCchan             RFlevelP;
  ADCchan             RFlevelN;
  ADCchan             DriveV;
  ADCchan             DriveI;
  // DAC channels
  DACchan             DCbias;  
  // Piece wise linear calibration data structures
  bool                UsePWL;           // True to use PWL table
  PWLcalibration      PWLcal[2];        // Piece wise linear data structures, 0 for RF+ and 1 of RF-
  // Ramp parameters
  float               rampStartV;
  float               rampEndV;
  float               rampTime;
  float               changePoint;
  float               rampMult;
  RampTriggerOptions  rampTriggerOptions;
  // Trigger parameters
  TriggerOptions      tiggerOptions;
  ActiveLevel         triggerLevel;
  //
  unsigned int  Signature;              // Must be 0xAA55A5A5 for valid data
} RFgenData;

typedef struct
{
  bool          update;
  // RFgen settings
  bool          Enable;
  int           Freq;                   // RFgen operating frequency
  float         Drive;
  float         Setpoint;               // RF voltage setpoint in volts
  bool          Mode;                   // Mode = true for closed loop, false for open
  bool          Gate;                   // True to enable gate mode
} State;


// Prototypes...
void ProcessSerial(bool scan = true);
void SaveSettings(void);
void RestoreSettings(void);
void Software_Reset(void);
void FormatFLASH(void);
void Debug(int i);
void ReadADC(int chan);
void RFdriver_tune(void);

void calBias(void);
void genPWLcalTable(char *phase);
void SetRelay(char *rly, char *state);
void GetRelay(int rly);
void SetBias(char *bias);
void calCurrent(void);

#ifdef RFGEN2
void setRampStartV(char *val);
void setRampEndV(char *val);
void setRampTime(char *val);
void setRampStart(void);
void setRampTrigger(char *val);
void getRampTrigger(void);
void setRampChangePoint(char *val);
void setRampMult(char *val);
#endif
void setTrigger(char *val);
void getTrigger(void);
void setTriggerLevel(char *val);
void getTriggerLevel(void);

void bootloader(void);
void saveDefaults(void);
void loadDefaults(void);
void saveCalibrations(void);
void loadCalibrations(void);

void myAnalogWriteDMA(int cnts);
void myAnalogWrite(int cnts);

float PWLlookup(int ph, int adcval);

#endif
