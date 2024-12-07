#include "si5351.h"
#include "RFgenerator.h"

#ifndef Hardware_h
#define Hardware_h

// DIO lines
#define RFON      7
#define STATUS_LT 9
#define PB        11
#define TRIG      2
#define PWRON     25
#define RFDISABLE 12

// Relay signals
#ifdef RFGEN2
#define RLY1      24
#define RLY2      5
#define RLY3      4
#define RLY4      3
#else
#define RLY1      5
#define RLY2      4
#define RLY3      3
#endif

// Analog IO
#define BIAS      A0
#define RFVP      A1
#define RFVN      A2
#define RFLVL     A3
#define SUPPLYV   A5

// Ethernet interface
// RX/TX used to communicate with interface
#define eCTRL     23

extern Si5351 si5351;

// M4 pin assigenments

#define   MAXDRIVE 999

typedef struct
{
  int8_t  Chan;                   // ADC channel number 0 through max channels for chip.
                                  // If MSB is set then this is a M0 ADC channel number
  float   m;                      // Calibration parameters to convert channel to engineering units
  float   b;                      // ADCcounts = m * value + b, value = (ADCcounts - b) / m
} ADCchan;

typedef struct
{
  int8_t  Chan;                   // DAC channel number 0 through max channels for chip
  float   m;                      // Calibration parameters to convert engineering to DAC counts
  float   b;                      // DACcounts = m * value + b, value = (DACcounts - b) / m
} DACchan;

// Function prototypes
int   GetADCvalue(int chan, int num);
float Counts2Value(int Counts, DACchan *DC);
float Counts2Value(int Counts, ADCchan *ad);
int   Value2Counts(float Value, DACchan *DC);
int   Value2Counts(float Value, ADCchan *ac);
float ReadADCchannel(ADCchan adch);

void ProgramFLASH(char * Faddress,char *Fsize);

void setupD10pwm();
void setD10pwm(int value);
void setFrequency(int);
void setDrive(float percent);


#endif
