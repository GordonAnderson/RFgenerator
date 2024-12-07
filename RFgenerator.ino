#include <Arduino.h>
#include <variant.h>
#include <wiring_private.h>
#include "SERCOM.h"
#include <Thread.h>
#include <ThreadController.h>
#include <Adafruit_DotStar.h>
#include <arduino-timer.h>
#include "si5351.h"

#include <Wire.h>
#include <SPI.h>
#include "Hardware.h"
#include "RFgenerator.h"
#include "ethernet.h"
#include "Errors.h"
#include "Serial.h"
#include "Button.h"
#include <Adafruit_SPIFlash.h>
#include <FlashStorage.h>
#include <FlashAsEEPROM.h>
#include <SerialBuffer.h>


#include "wiring_private.h"

//
// Gordon Anderson
//
// The RFgenerator hardware uses an Adafruit ItsyBitsy M4 processor. The RF generator is a standalone
// RF driver with both USB and eternet interfaces. The system has the follwoing features:
//  1.) LED indicators:
//      - RED    = RF on
//      - Yellow = status
//                 flash when tuning
//                 fast flash when limit exceeded
//  2.) Puush button control
//      - Push once short, RF on or off
//      - Push and hold = tune
//  3.) External serial port interface, USB virtual com port
//  4.) RF level lookup table
//  5.) Gate input, TTL
//  6.) Asigmnable
//  7.) Power and drive limits
//
// Rev 2 RF generator main board updates:
//  - Uses si5351 PLL
//  - Has 4 total relays to allow symetrical inductance switching
//  - Bias supply used to control the RF level FET
//  - Supports DC bias external input or RF common mode drive input
//  - Software updates:
//    - Add gate input with level control
//    - Add RF ramp capability, ramp on trigger input edge, pos or neg
//      ramp using bias, ramp up or down, definable ramp rate
//      Could calibrate the bias to RF level. Do this after level calibration.
//      Calibrate bias to FET voltage output
//    - Support fast RF off, change frequency, adds latency
//    - Design details:
//        - Calibrate bias to gate drive voltage, use bias command as is
//        - Make lookup table from bias to RF level, RF+,RF-,RF for average
//        - RF level mode using FET driver
//        - Closed loop mode using bias to RF level. 
//        - Automatically control drive to set FET voltage above bias level by a few volts
//          may want to servo this parameter
//        - Ramp parameters
//            Ramp time in mS
//            RF start level
//            RF stop level
//        - Initially just ramp the bias value from its current setting to the 
//          given value. In this case you don't even indicate direction. Here are
//          initial commands:
//            SGENA,POS | NEG | OFF
//            GGENA
//            SRTRG,POS | NEG | OFF
//            GRTRG
//            SRV,voltage
//            GRV
//            SRTIME,time   // Ramp time in mS, 100 maximum
//            GRTIME
//            This is the fast off option and quenches the RF my changing the drive freq,
//            this does add lantency
//            SFOENA,TRUE | FALSE, frequency
//            GFOENA
//            This command will stop the osc and set drive to 0 when gating, adds latency
//            SQUIET,TRUE | FALSE
//            GQUIET
//          Note; when disabled, after ramping is complete, turn off the oscilator and 
//          return drive to 0 if in quiet mode.
//          When enabled, turn on osc, turn on drive, set bias value to values when gated off.
//
//  August 8, 2024
//      - Add command to turn off the oscillator
//      - Calibrate the Bias to voltage at the driver, after the drive voltage control FET
//      - Calibrate Bias voltage to RF level?
//      - To define a ramp range we need the start and stop RF levels and we need to map them to
//        there respective Bias voltages. RF drive voltage must exceed Bias voltage
//      - The ramping function in the RF generator
//        - Set Drive
//        - Set start Bias
//        - Set end Bias
//        - Calculate step value
//        - Start via command or after trigger with a delay
//        - On trigger set to start Bias and after delay start ramping
//      - In a script in MIPS host app that talks to both generators
//        - System setup/calibration
//          - Set main RF drive to 0
//          - Set Bias on aux RF to 0
//          - Set aux RF drive to 100 percent
//          - Set Bias to 5V and measure RF level on main RF
//          - Set Bias to 10V and measure RF level on main RF
//          - Calculate Bias voltage to RF level
//          - Set Bias to high RF ramp level
//          - Calculate required drive %
//          - Set up the aux RF parameters
// August 11,2024
//    - Change default cal values for BIAS when building RFGEN2, done
//    - Look for bug in auto tune, fixed I think!
//    - Allow changing auto tune frequency limits, done
//    - Add tune status command, done
//    - Set BIAS in tune for RFGEN2, done
//    - Fix gate funtion using gate global variable
//      - Not sure how to use this
//    - Hardware update
//      - fix divider on FET gate
//      - Add filter cap (100pF) to bias amp, remove ref filter cap
//      - Add pull up on the trigger input signal to fet faster response
//
//  Version history:
//    Version 1.0, April 25, 2022
//      - Orginal version
//    Version 1.1, May 5, 2022
//      - Lowered low freq limit to 400KHz
//    Version 1.2, April 27, 2023
//      - Improved startup
//    Version 1.3, Aug 21, 2023
//      - Improved startup, it was locking up trying to turn off the PLL with no power on the chip
//
// Coil details for first unit:
//   - Form 0.5"
//   - 16 gauge magnet wire
//   - 1st long section 2.5"
//   - All remaining sections (4 total) 0.55" long
//   - Monitor caps are 1500pF
//   - With two BNC shorting caps for 750pF across coil
//       - no relays 1.76MHz
//       - Relay 1          = 1.9MHz
//       - Relays 1 & 2     = 2.1MHz
//       - Relays 1 & 2 & 3 = 2.21MHz

// PLL is not stable below 250,000Hz
#define MinFreq    400000
#define MaxFreq    5000000

float   currentBias = 0;

//#define writeBIAS  analogWrite(rfgendata.DCbias.Chan,Value2Counts(currentBias,&rfgendata.DCbias))

#define writeBIAS  myAnalogWrite(Value2Counts(currentBias,&rfgendata.DCbias));

void myAnalogWrite(int cnts)
{
    noInterrupts();
    while (DAC->SYNCBUSY.bit.DATA0);
		DAC->DATA[0].reg = cnts;
    while (DAC->SYNCBUSY.bit.DATA0);
    interrupts();
}  

const char   Version[] PROGMEM = "MIPS RF Genrator Version 1.3, Aug 21, 2023";
RFgenData   rfgendata;

RFgenData Rev_1_rfgendata = {
                            sizeof(RFgenData),"RFgen",1,
                            false,
                            0x58,
                            1728000,0,500,0,0,false,false,
                            0.005,
                            60,25,
                            A1,.6184,-156.32,
                            A2,.6184,-156.32,
                            A3,118.29,0,
                            A4,138.5,430,
                            #ifdef RFGEN2
                            A0,36.72,1992.22,
                            #else
                            A0,36.76,1955.88,
                            #endif
                            true,
                            9,164,206,334,403,452,494,542,585,630,894,4,23,57,98,138,184,224,266,306,894,
                            9,164,206,334,403,452,494,542,585,630,894,4,23,57,98,138,184,224,266,306,894,
                            10,0,2,RAMP_TRIG_OFF,
                            TRIG_OFF,ACTIVE_HIGH,
                            SIGNATURE
                          };

// Ramp parameters
#define RAMPLOOPTIME  0.0010
int   rampLoops;
float rampStepSize;

// System variables
State state;
float RFvoltageP = 0;
float RFvoltageN = 0;
float DriveVolt = 0;
float DriveCurrent = 0;
float Power = 0;

auto timer = timer_create_default();

// Auto tune parameters
bool TuneRequest   = false;
bool RetuneRequest = false;
bool Tuning        = false;
bool TuneReport    = false;

int  MaxTuneFreq = MaxFreq;
int  MinTuneFreq = MinFreq;
// Tune states
#define TUNE_SCAN_DOWN 1
#define TUNE_SCAN_UP 2

#define MaxNumDown 5
#define MAXSTEP    100000

Button functionPin;

// for flashTransport definition
#include "flash_config.h"
Adafruit_SPIFlash flash(&flashTransport);
// file system object from SdFat
FatVolume fatfs;
File32 file;

// Reserve a portion of flash memory to store configuration
// Note: the area of flash memory reserved is cleared every time
// the sketch is uploaded on the board.
FlashStorage(flash_rfgendata, RFgenData);

// ThreadController that will control all threads
ThreadController control = ThreadController();
//Threads
Thread SystemThread = Thread();

void Control(void)
{
  if((rfgendata.Mode) && (rfgendata.Enable))
  {
    // Here if in closed loop control mode.
    float error = rfgendata.Setpoint - (RFvoltageP + RFvoltageN)/2;
    rfgendata.Drive += error * rfgendata.Gain;
    if(rfgendata.Drive < 0) rfgendata.Drive=0;
  }
  // Limit testing
  if(rfgendata.Drive > rfgendata.MaxDrive) rfgendata.Drive = rfgendata.MaxDrive;
  if(Power > rfgendata.MaxPower) rfgendata.Drive -= 0.1;
  if(rfgendata.Drive < 0) rfgendata.Drive = 0;
}

// This function is called at 40 Hz
void Update(void)
{
  float val;

// Update all changes
  if((state.Enable != rfgendata.Enable) || (state.update))
  {
    state.Enable = rfgendata.Enable;
    if(rfgendata.Enable)
    {
      setFrequency(rfgendata.Freq);
      delay(100);
      setDrive(rfgendata.Drive);
    }
    else
    {
      setDrive(0);
      delay(100);
      FS7140setup(rfgendata.TWIadd,rfgendata.Freq,true);
    }
  }
  if((state.Freq != rfgendata.Freq) || (state.update))
  {
    state.Freq = rfgendata.Freq;
    if(rfgendata.Enable) setFrequency(rfgendata.Freq);
  }
  if((state.Drive != rfgendata.Drive) || (state.update))
  {
    state.Drive = rfgendata.Drive;
    if(rfgendata.Enable) setDrive(rfgendata.Drive);
  }
  if((state.Setpoint != rfgendata.Setpoint) || (state.update))
  {
    state.Setpoint = rfgendata.Setpoint;
  }
  if((state.Mode != rfgendata.Mode) || (state.update))
  {
    state.Mode = rfgendata.Mode;
  }
  if((state.Gate != rfgendata.Gate) || (state.update))
  {
    state.Gate = rfgendata.Gate;
    if(rfgendata.Gate) digitalWrite(RFDISABLE,HIGH);
    else digitalWrite(RFDISABLE,LOW);
  }
  state.update = false;
// Read the ADC values and convert
  if(!rfgendata.UsePWL)
  {
    val = ReadADCchannel(rfgendata.RFlevelP);
    RFvoltageP = (1.0 - FILTER) * RFvoltageP + FILTER * val;
    val = ReadADCchannel(rfgendata.RFlevelN);
    RFvoltageN = (1.0 - FILTER) * RFvoltageN + FILTER * val;  
  }
  else
  {
    RFvoltageP = PWLlookup(0,GetADCvalue(rfgendata.RFlevelP.Chan,20));
    RFvoltageN = PWLlookup(1,GetADCvalue(rfgendata.RFlevelN.Chan,20));
  }
  val = ReadADCchannel(rfgendata.DriveV);
  DriveVolt = (1.0 - FILTER) * DriveVolt + FILTER * val;
  val = ReadADCchannel(rfgendata.DriveI);
  if(!rfgendata.Enable) val = 0;
  DriveCurrent = (1.0 - FILTER) * DriveCurrent + FILTER * val;
//  Power = DriveVolt * DriveCurrent;
  Power = 24 * DriveCurrent;
// Control functions
  RFdriver_tune();
  Control();
// Check limits and apply
  if(rfgendata.Drive > rfgendata.MaxDrive) rfgendata.Drive = rfgendata.MaxDrive;
}

bool RFonLED(void *)
{
  // If tuning then flash the LED
  // If RF on turn on the LED
  // If RF off tuen off the LED
  //timer.at(500, RFonLED);
  if(rfgendata.Enable) digitalWrite(RFON,LOW);
  else digitalWrite(RFON,HIGH);
  return true;
}

bool StatusLED(void *)
{
  if(Tuning)
  {
    if(digitalRead(STATUS_LT) == HIGH) digitalWrite(STATUS_LT,LOW);
    else digitalWrite(STATUS_LT,HIGH);
    return true;
  }
  digitalWrite(STATUS_LT,HIGH);
  return true;
}

void configureRelays(void)
{
   if((rfgendata.Relay & 0x01) != 0) digitalWrite(RLY1, HIGH);
   else digitalWrite(RLY1, LOW);
   if((rfgendata.Relay & 0x02) != 0) digitalWrite(RLY2, HIGH);
   else digitalWrite(RLY2, LOW);
   if((rfgendata.Relay & 0x04) != 0) digitalWrite(RLY3, HIGH);
   else digitalWrite(RLY3, LOW);
   #ifdef RFGEN2
   if((rfgendata.Relay & 0x08) != 0) digitalWrite(RLY4, HIGH);
   else digitalWrite(RLY4, LOW);
   #endif
}

void bootloader(void)
{
  __disable_irq();
 	//THESE MUST MATCH THE BOOTLOADER
	#define DOUBLE_TAP_MAGIC 			      0xf01669efUL
	#define BOOT_DOUBLE_TAP_ADDRESS     (HSRAM_ADDR + HSRAM_SIZE - 4)

	unsigned long *a = (unsigned long *)BOOT_DOUBLE_TAP_ADDRESS;
	*a = DOUBLE_TAP_MAGIC;
	//NVMCTRL->ADDR.reg  = APP_START;
	//NVMCTRL->CTRLB.reg = NVMCTRL_CTRLB_CMD_EB | NVMCTRL_CTRLB_CMDEX_KEY;
	
	// Reset the device
	NVIC_SystemReset() ;

	while (true);
}


// Functions supporting Circuit Python file system (CPFS). This is used to save
// the configuration and calibration data to the SPI flash filesystem.
// Provides non volitial storage of setup data.
bool FSsetup(void)
{
  // Init external flash
  if (!flash.begin()) serial->println("Error, failed to initialize flash chip!");
  else
  {
    serial->println("Flash chip initalized!");
    if(!fatfs.begin(&flash)) serial->println("Failed to mount filesystem!");
    else
    {
      serial->println("Mounted filesystem!");
      return true;
    }
  }
  return false;
}
void saveDefaults(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_WRITE | O_CREAT))==0) serial->println("Can't create default.dat!");
  else
  {
    size_t num = file.write((void *)&rfgendata,sizeof(RFgenData));
    file.close();
    serial->print("default.dat written, number of bytes = ");
    serial->println(num);
  }
}
void loadDefaults(void)
{
  RFgenData h;

  if(!FSsetup()) return;
  if((file = fatfs.open("default.dat",O_READ))==0) serial->println("Can't open default.dat!");
  else
  {
    size_t num = file.read((void *)&h,sizeof(RFgenData));
    file.close();
    if((num != sizeof(RFgenData)) || (h.Signature != SIGNATURE))
    {
      serial->println("Error reading default.dat file!");
      return;
    }
    rfgendata = h;
    serial->print("default.dat read, number of bytes = ");
    serial->println(num);
  }
}
void saveCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_WRITE | O_CREAT))==0) serial->println("Can't create cal.dat!");
  else
  {
    size_t num = file.write((void *)&rfgendata.RFlevelP,sizeof(ADCchan));
    num += file.write((void *)&rfgendata.RFlevelN,sizeof(ADCchan));
    num += file.write((void *)&rfgendata.DriveV,sizeof(ADCchan));
    num += file.write((void *)&rfgendata.DriveI,sizeof(ADCchan));
    num += file.write((void *)&rfgendata.DCbias,sizeof(DACchan));
    num += file.write((void *)rfgendata.PWLcal,sizeof(PWLcalibration) * 2);
    file.close();
    serial->print("cal.dat written, number of bytes = ");
    serial->println(num);
  }
}
void loadCalibrations(void)
{
  if(!FSsetup()) return;
  if((file = fatfs.open("cal.dat",O_READ))==0) serial->println("Can't open cal.dat!");
  else
  {
    size_t num = file.read((void *)&rfgendata.RFlevelP,sizeof(ADCchan));
    num += file.read((void *)&rfgendata.RFlevelN,sizeof(ADCchan));
    num += file.read((void *)&rfgendata.DriveV,sizeof(ADCchan));
    num += file.read((void *)&rfgendata.DriveI,sizeof(ADCchan));
    num += file.read((void *)&rfgendata.DCbias,sizeof(DACchan));
    num += file.read((void *)rfgendata.PWLcal,sizeof(PWLcalibration) * 2);
    file.close();
    serial->print("cal.dat read, number of bytes = ");
    serial->println(num);
  }
}
// End of CPFS

void setup() 
{
  setupD10pwm();
  setD10pwm(0);
  pinMode(PWRON,INPUT);
  if(digitalRead(PWRON) == LOW)
  {
    while(digitalRead(PWRON) == LOW) delay(100);
    delay(100);
    Software_Reset();
  }
  
//  while analogRead(SUPPLYV)
  // Turn off all the relays
  digitalWrite(RLY1,LOW);
  pinMode(RLY1,OUTPUT);
  digitalWrite(RLY2,LOW);
  pinMode(RLY2,OUTPUT);
  digitalWrite(RLY3,LOW);
  pinMode(RLY3,OUTPUT);
  #ifdef RFGEN2
  digitalWrite(RLY4,LOW);
  pinMode(RLY4,OUTPUT);
  #endif
  digitalWrite(RFDISABLE,LOW);
  pinMode(RFDISABLE,OUTPUT);
  pinMode(TRIG,INPUT_PULLUP);
   // Read the flash config contents and test the signature
  rfgendata = flash_rfgendata.read();
  if(rfgendata.Signature != SIGNATURE) rfgendata = Rev_1_rfgendata;
  // Init serial communications
  configureRelays();
  SerialInit();
  Serial1.begin(115200); // external RS232 port
  // Init the TWI interface
  Wire.begin();
  #ifdef RFGEN2
  Wire.setClock(400000);
  #endif
  // Init the driver timer
  setupD10pwm();
  // Init the DIO
  // This disables the  drive
  // This disables the RF
  analogReadResolution(12);
  analogWriteResolution(12);
  currentBias = rfgendata.Bias;
  analogWrite(rfgendata.DCbias.Chan,Value2Counts(currentBias,&rfgendata.DCbias));
  writeBIAS;
  // Configure Threads
  SystemThread.setName((char *)"Update");
  SystemThread.onRun(Update);
  SystemThread.setInterval(25);
  // Add threads to the controller
  control.add(&SystemThread);
   // Set the drive
  setDrive(0);
 // Set the frequency
  setFrequency(rfgendata.Freq);
  state.update = true; // force full update.
  functionPin.begin(PB);
  // Start connect status LED
  pinMode(PB,INPUT_PULLUP);
  pinMode(RFON,OUTPUT);
  digitalWrite(RFON,HIGH);
  pinMode(STATUS_LT,OUTPUT);
  digitalWrite(STATUS_LT,HIGH);
  timer.every(500, RFonLED);
  timer.every(500, StatusLED);
  rfgendata.Enable=false;
  functionPin.released();
  delay(4000);
  Ethernet_init();
}

// This function process all the serial IO and commands
void ProcessSerial(bool scan)
{
  // Put serial received characters in the input ring buffer
  if (Serial.available() > 0)
  {
    serial = &Serial;
    PutCh(Serial.read());
  }
  if (Serial1.available() > 0)
  {
    serial = &Serial1;
    PutCh(Serial1.read());
  }
  if (!scan) return;
  // If there is a command in the input ring buffer, process it!
  if (RB_Commands(&RB) > 0) while (ProcessCommand() == 0); // Process until flag that there is nothing to do
}

// Auto tune algorithm, procedure is as follows:
// 1.) Set power tp 10%
// 2.) Set frequency to 1MHz
// 3.) Go down in frequency in 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 4.) Go up in frequency from 1MHzin 100KHz steps and record amplitude, stop when 5 steps in a row are all decreasing
// 5.) Use the peak found in steps 3 and 4 and sweep in 10K steps using the same procedure in 3 and 4
// 6.) Use the peak found in step 5 and sweep in 1K steps using the same procedure in 3 and 4
// 7.) Done!
//
// Called from the main processing loop, this function does not block, uses a state machine to perform the logic
// States
//  TUNE_SCAN_DOWN
//  TUNE_SCAN_UP

void RFdriver_tune(void)
{
   static int    TuneStep = 100000, TuneState;
   static float  Max, Current, Last;
   static int    FreqMax, Freq;
   static int    NumDown,Nth;

   if(TuneRequest)
   {
     // Correct errors in tune parameters if needed
     if(MaxTuneFreq > MaxFreq) MaxTuneFreq = MaxFreq;
     if(MaxTuneFreq < MinFreq) MaxTuneFreq = MinFreq;
     if(MinTuneFreq > MaxFreq) MinTuneFreq = MaxFreq;
     if(MinTuneFreq < MinFreq) MinTuneFreq = MinFreq;
     if(MinTuneFreq > MaxTuneFreq) MinTuneFreq = MaxTuneFreq;
     rfgendata.Mode = false; // Make sure we are in open loop mode!
     // Set freq to 1MHz
     rfgendata.Freq = 1000000;
     // Set drive to 10%
     rfgendata.Enable = true;
     rfgendata.Drive = 10;
     #ifdef RFGEN2
     currentBias = rfgendata.Bias = 12;
     writeBIAS;
     #endif
     Tuning = true;
     TuneStep = MAXSTEP;
     Freq = 1000000;
     Last = Max = 0;
     NumDown = -MaxNumDown;
     TuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     TuneReport = true;
     if(TuneReport) serial->println("Tuning...");
     return;
   }
   if(RetuneRequest)
   {
     rfgendata.Mode = false; // Make sure we are in open loop mode!
     // Set freq to current
     Freq = rfgendata.Freq;
     Tuning = true;
     TuneStep = 1000;
     Last = Max = 0;
     NumDown = 0;
     RetuneRequest = false;
     TuneState = TUNE_SCAN_DOWN;
     Nth = 20;
     if(TuneReport) serial->println("Re-tuning...");
     return;
   }
   if(!Tuning) return;
   if(--Nth > 0) return;
   Nth = 20;
   // Here if the system is tuning
   Current = RFvoltageP + RFvoltageN;
   switch (TuneState)
   {
     case TUNE_SCAN_DOWN:
//        if(Current > Max)
//        {
//          Max = Current;
//          FreqMax = fragdata.Freq;
//        }
        if(Current <= (Last + 1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        rfgendata.Freq -= TuneStep;
        if((NumDown >= MaxNumDown) || (rfgendata.Freq < MinTuneFreq))
        {
          TuneState = TUNE_SCAN_UP;
          rfgendata.Freq = Freq;
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        if(Current > Max)  // Move this code here to force tune system to not use the limit value
        {
          Max = Current;
          FreqMax = rfgendata.Freq;
        }
        break;
     case TUNE_SCAN_UP:
        if(Current > Max)
        {
          Max = Current;
          FreqMax = rfgendata.Freq;
        }
        if(Current <= (Last +1)) NumDown++;
        else 
        {
          NumDown = 0;
          if(TuneStep == MAXSTEP) NumDown = -MaxNumDown;
        }
        rfgendata.Freq += TuneStep;
        if((NumDown >= MaxNumDown) || (rfgendata.Freq > MaxTuneFreq))
        {
          // Here we have found the peak for this step size, this
          // process repeats until step size is 1KHz
          Freq = FreqMax;
          if(Freq < MinTuneFreq) Freq = MinTuneFreq;
          if(Freq > MaxTuneFreq) Freq = MaxTuneFreq;
          rfgendata.Freq = Freq;
          if(TuneStep == 1000)
          {
            // If here we are done!
            Tuning = false;
            if(TuneReport && !SerialMute)
            {
              serial->print("Auto tune complete, frequency = ");
              serial->println(Freq);
            }
            TuneReport = false;
            return;
          }
          else 
          {
            if(TuneStep == MAXSTEP) TuneStep = 10000;
            else TuneStep /= 10;
          }
          TuneState = TUNE_SCAN_DOWN;
          NumDown = 0;
        }
        break;
     default:
        break;
   }
   Last = Current;
}

void loop() 
{
  static uint32_t now = millis();
  static uint32_t downTime, timeHeld = 0;
  static bool isDown = false;
  
  noInterrupts();
  if(currentBias > 0) writeBIAS;      // This causes 10uS interrupt jitter on IO lines
  interrupts();

  if(digitalRead(PWRON) == LOW) 
  {
    setD10pwm(0);
    //FS7140setup(rfgendata.TWIadd,rfgendata.Freq,true);   // This will lock up without power!
    //delay(1000);
    //while(digitalRead(PWRON) == LOW);
    Software_Reset();
    while(true);      // Should never get here
  }
  
  ProcessSerial();
  control.run();
  timer.tick();

  if(now != millis())
  {
    now = millis();
    if(functionPin.down())
    {
      if(!isDown)
      {
        isDown = true;
        downTime = now;
      }
      // Record the length to time in mS the button is down.
      // Process when the button is released
      timeHeld = now - downTime;
    }
    else isDown = false;
    if(functionPin.released())
    {
      if(timeHeld >= 3000)
      {
        // Start a tune cycle
        TuneRequest = true;
      }
      else
      {
        // Cycle RF on/off state
        if(rfgendata.Enable) rfgendata.Enable = false;
        else rfgendata.Enable = true;
      }
    }
  }
}

//
// Host command functions
//

// Range test user entered value and update variable is valid value is entered.
// Return true is value value, retune false on error and send NAK to host
bool valueUpdate(float *fval, char *cval, float LL, float UL)
{
  float   val;
  String  token = cval;

  val = token.toFloat();
  if((val > UL) || (val < LL)) 
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  *fval = val;
  return true;
}

bool valueUpdate(bool *bval, char *cval)
{
  bool    v;
  String  token = cval;

  if(token == "TRUE") v = true;
  else if(token == "FALSE") v = false;
  else
  {
    SetErrorCode(ERR_BADARG); 
    SendNAK;
    return false;
  }
  *bval = v;
  return true;
}

void SaveSettings(void)
{
  rfgendata.Signature = SIGNATURE;
  flash_rfgendata.write(rfgendata);
  SendACK;
}

void RestoreSettings(void)
{
  static RFgenData fd;
  
  // Read the flash config contents and test the signature
  fd = flash_rfgendata.read();
  if(fd.Signature == SIGNATURE) rfgendata = fd;
  else
  {
    SetErrorCode(ERR_EEPROMWRITE);
    SendNAK;
    return;
  }
  SendACK;  
}

void Software_Reset(void)
{
  NVIC_SystemReset();  
  while(1);
}

void FormatFLASH(void)
{
  flash_rfgendata.write(Rev_1_rfgendata);  
  SendACK;
}

void ReadADC(int chan)
{
  SendACKonly;
  serial->println(GetADCvalue(chan, 20));
}

void Debug(int i)
{
  float biasV;

				//while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				//DAC->CTRLA.bit.ENABLE = 0;     // disable DAC

				//while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				//DAC->DACCTRL[0].bit.ENABLE = 1;

				//while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				//DAC->CTRLA.bit.ENABLE = 1;     // enable DAC

if(i>0)
{
				//while ( !DAC->STATUS.bit.READY0 );

				//while (DAC->SYNCBUSY.bit.DATA0);
				//DAC->DATA[0].reg = 1234;  



        while ( !DAC->STATUS.bit.READY0 );

				while (DAC->SYNCBUSY.bit.DATA0);
				DAC->DATA[0].reg = i;
        return;
}

        serial->println(DAC->CTRLB.reg,HEX);
        serial->println(DAC->DACCTRL[0].reg,HEX);

				while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				DAC->CTRLA.bit.ENABLE = 0;     // disable DAC
        delayMicroseconds(100);

				while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
        DAC->DACCTRL[0].reg = 0x020A;
				while (DAC->SYNCBUSY.bit.ENABLE || DAC->SYNCBUSY.bit.SWRST);
				DAC->CTRLA.bit.ENABLE = 1;     // enable DAC


        return;


  Wire.setClock(400000);
  if(i==-1)
  {
    // Enable clock
    si5351.output_enable(SI5351_CLK0, 1);
    return;
  }
  if(i==-2)
  {
    // Disable clock
    si5351.output_enable(SI5351_CLK0, 0);
    return;
  }
  if(i==-3)
  {
    // Pulse clock on for 1mS
    //si5351.output_enable(SI5351_CLK0, 1);
    rfgendata.Bias = 20;
    writeBIAS;
    delay(1);
    setFrequency(4000000);
    delay(1);
    rfgendata.Bias = 0;
    writeBIAS;
    setFrequency(rfgendata.Freq);

    //si5351.set_clock_invert(SI5351_CLK0, 1);
    //si5351.output_enable(SI5351_CLK0, 0);
    //si5351.set_clock_invert(SI5351_CLK0, 0);
   return;
  }
  if(i==-4)
  {
    // Pulse output on for 1mS
    for(i=0;i<2000;i++)
    {
      rfgendata.Bias = 20;
      writeBIAS;
      delay(1);
      rfgendata.Bias = 0;
      writeBIAS;
      delay(2);
    }
   return;
  }
  //rfgendata.Bias = i;
  //writeBIAS;
  //return;
  // This loop takes 30mS to execute when i = 1, this is 4000 loops or 7.5uS per loop
  // For a linear ramp could first calculate the DAC count delta per step and speed this
  // loop up, it could be over a factor of 2.
  for(biasV=40;biasV>0;biasV-=(float)i/100.0)
  {
    rfgendata.Bias = biasV;
    writeBIAS;
  }
}

//
// The following functions support the piece wise linear calibration model.
//

// These functions will generate the calibration table. The user will adjust the 
// drive to set a desired voltage and then enter the value. Up to ten points
// be definded. Enter an empty string to terminate data entry. It is assumed 
// that the channel has been tuned and is properly connected.
// channel is 1 through number of channels
// phase is RF+ or RF-
uint8_t PWLch;

void RFdriveAllowADJ(void)
{
  // Call the task system to allow all system processing
  control.run();
}

void genPWLcalTable(char *phase)
{
  String sToken;
  char   *res;
  int    brd,ph,i;
  float  Drive;
  
  sToken = phase;
  if((sToken != "RF+") && (sToken != "RF-")) BADARG;
  if(sToken == "RF+") ph = 0;
  else ph = 1;
  // Send the user instructions.
  serial->println("This function will generate a piecewise linear");
  serial->println("calibration table. Set the drive level to reach");
  serial->println("desired calibration points and then enter the");
  serial->println("measured value to the nearest volt. Press enter");
  serial->println("When finished. Voltage must be increasing!");
  // Loop to allow user to adjust drive and enter measured voltage
  rfgendata.PWLcal[ph].num = 0;
  i=0;
  Drive = rfgendata.Drive;
  while(true)
  {
     serial->print("\nPoint ");
     serial->println(i+1);
     res = UserInput((char *)"Set drive level, %: ", RFdriveAllowADJ);
     if( res == NULL) break;
     sToken = res;
     rfgendata.Drive = sToken.toFloat();
     res = UserInput((char *)"\nEnter measured voltage: ", RFdriveAllowADJ);
     if( res == NULL) break;
     sToken = res;
     rfgendata.PWLcal[ph].Value[i] = sToken.toInt();
     // Read the ADC raw counts
     if(ph == 0) rfgendata.PWLcal[ph].ADCvalue[i] = GetADCvalue(rfgendata.RFlevelP.Chan,20);
     else rfgendata.PWLcal[ph].ADCvalue[i] = GetADCvalue(rfgendata.RFlevelN.Chan,20);
     i++;
     rfgendata.PWLcal[ph].num = i;
     if(i>=MAXPWL) break;
  }
  serial->println("");
  // Report the table
  serial->print("Number of table entries: ");
  serial->println(rfgendata.PWLcal[ph].num);
  for(i=0;i<rfgendata.PWLcal[ph].num;i++)
  {
    serial->print(rfgendata.PWLcal[ph].Value[i]);
    serial->print(",");
    serial->println(rfgendata.PWLcal[ph].ADCvalue[i]);
  }
  // Done!
  serial->println("\nData entry complete!");
  rfgendata.Drive = Drive;
}

// This function will use the piecewise linear table to convert the 
// adcval to output voltage.
// ph = phase, 0 = RF+, 1 = RF-
float PWLlookup(int ph, int adcval)
{
  int            brd,i;
  PWLcalibration *pwl;
  
  pwl = &rfgendata.PWLcal[ph];
  if(pwl->num < 2) return 0;
  for(i=0;i<pwl->num-1;i++)
  {
    if(adcval < pwl->ADCvalue[i]) break;
    if((adcval >= pwl->ADCvalue[i]) && (adcval <= pwl->ADCvalue[i+1])) break;
  }
  if(i == pwl->num-1) i--;
  // The points at i and i+1 will be used to calculate the output voltage
  // y = y1 + (x-x1) * (y2-y1)/(x2-x1)
  return (float)pwl->Value[i] + ((float)adcval - (float)pwl->ADCvalue[i]) * ((float)pwl->Value[i+1]-(float)pwl->Value[i])/((float)pwl->ADCvalue[i+1] -(float)pwl->ADCvalue[i]);
}

void SetRelay(char *rly, char *state)
{
  String token;
  int    i,msk;

  if(rfgendata.Enable) BADARG;
  token = rly;
  i = token.toInt();
  token = state;
  if(i==1)      msk = 0x01;
  else if(i==2) msk = 0x02; 
  else if(i==3) msk = 0x04;
  #ifdef RFGEN2
  else if(i==4) msk = 0x08;
  #endif
  else BADARG;
  if((token!="OPEN") && (token!="CLOSE")) BADARG;
  if(token=="CLOSE") rfgendata.Relay |= msk;
  else rfgendata.Relay &= ~msk;
  configureRelays();
  SendACK;
}

void GetRelay(int rly)
{
  #ifdef RFGEN2
  if((rly<1) || (rly>4)) BADARG;
  #else
  if((rly<1) || (rly>3)) BADARG;
  #endif
  SendACKonly;
  if((rfgendata.Relay & (1 << (rly-1))) != 0) serial->println("CLOSE");
  else serial->println("OPEN");
}

void SetBias(char *bias)
{
  #ifdef RFGEN2
  if(!valueUpdate(&rfgendata.Bias, bias, 0, 24)) return;
  #else
  if(!valueUpdate(&rfgendata.Bias, bias, -50, 50)) return;
  #endif
  currentBias = rfgendata.Bias;
  writeBIAS;
  SendACK;
}

void calBias(void)
{
  float V1,V2;
  int   C1,C2;

  #ifdef RFGEN2
  // Send the user instructions.
  serial->println("This function will generate the calibration");
  serial->println("parameters for the DC bias supply. Measure");
  serial->println("the DC voltage acress R39.");
  UserInput((char *)"Press enter to continue...",RFdriveAllowADJ);

  // Turn off the oscilator and set the drive to 100%
  si5351.output_enable(SI5351_CLK0, 0);
  setDrive(100);

  C1 = 2000;
  C2 = 2500;
  #else
  // Send the user instructions.
  serial->println("This function will generate the calibration");
  serial->println("parameters for the DC bias supply. Make sure");
  serial->println("the RF is diabled and measure the DC voltage");
  serial->println("on the RF+ output.");
  UserInput((char *)"Press enter to continue...",RFdriveAllowADJ);

  C1 = 2000;
  C2 = 3500;
  #endif
  analogWrite(rfgendata.DCbias.Chan,C1);
  V1 = UserInputFloat((char *)"Enter measured voltage: ", RFdriveAllowADJ);

  analogWrite(rfgendata.DCbias.Chan,C2);
  V2 = UserInputFloat((char *)"Enter measured voltage: ", RFdriveAllowADJ);

  // Calculate the calibration parameters
  // C = V * m + b
  // C1-C2 = (V1 - V2) * m
  // m = (C1 - C2)/(V1 - V2)
  // b = C - V * m
  rfgendata.DCbias.m = (float)(C1 - C2) / (V1 - V2);
  rfgendata.DCbias.b = (float)C1 - (V1 * rfgendata.DCbias.m);
  serial->print("Slope, "); serial->println(rfgendata.DCbias.m);
  serial->print("Intercept, "); serial->println(rfgendata.DCbias.b);
  #ifdef RFGEN2
  // Set the drive to 0 and turn on the oscillator
  setDrive(0);
  delay(100);
  si5351.output_enable(SI5351_CLK0, 1);
  #endif
  SendACK;
}

void calCurrent(void)
{
  serial->println("Calibrate current sensor.");
  // Set first drive level and ask for the current value
  rfgendata.Drive  = UserInputFloat((char *)"Enter drive level 1 : ", RFdriveAllowADJ);
  float cur1 = UserInputFloat((char *)"Enter current, amps : ", RFdriveAllowADJ);
  int   adc1 = GetADCvalue(rfgendata.DriveI.Chan,100);
  // Set second drive level and ask for the current value
  rfgendata.Drive = UserInputFloat((char *)"Enter drive level 2 : ", RFdriveAllowADJ);
  float cur2 = UserInputFloat((char *)"Enter current, amps : ", RFdriveAllowADJ);
  int   adc2 = GetADCvalue(rfgendata.DriveI.Chan,100);
  // Calculate the calibration parameters and apply.
  // counts = value * m + b
  // adc1 = cur1 * m + b
  // adc2 = cur2 * m + b
  // adc1 - adc2 = (cur1 - cur2) * m
  // m = (adc1 - adc2) / (cur1 - cur2)
  // b = adc2 - cur2 * m
  rfgendata.Drive = 10;
  rfgendata.DriveI.m = (float)(adc1 - adc2) / (cur1 - cur2);
  rfgendata.DriveI.b = (float)adc2 - cur2 * rfgendata.DriveI.m;
  serial->print("m = "); serial->println(rfgendata.DriveI.m); 
  serial->print("b = "); serial->println(rfgendata.DriveI.b); 
}

#ifdef RFGEN2
void prepairRamp(void)
{
  // Set drive level to support higest ramp level
  float maxV = rfgendata.rampStartV;
  if(rfgendata.rampEndV > maxV) maxV = rfgendata.rampEndV;
  rfgendata.Drive = (maxV / 24) * 100 + 5;
  if(rfgendata.Drive > 100) rfgendata.Drive = 100;
  // Set initial level
  currentBias = rfgendata.rampStartV;
  // Calculate loop steps and step size
  rampLoops = rfgendata.rampTime / RAMPLOOPTIME;
  rampStepSize = (rfgendata.rampEndV - rfgendata.rampStartV) / rampLoops;
  if(rfgendata.Enable)
  {
    setDrive(rfgendata.Drive);
  }
}
void executeRamp(void)
{
  for(int i=0;i<rampLoops;i++)
  {
    currentBias += rampStepSize;
    writeBIAS;
  }
}

void setRampStartV(char *val) { if(valueUpdate(&rfgendata.rampStartV, val, 0, 24)) SendACK;}
void setRampEndV(char *val)   { if(valueUpdate(&rfgendata.rampEndV, val, 0, 24))   SendACK;}
void setRampTime(char *val)   { if(valueUpdate(&rfgendata.rampTime, val, 0.5, 10)) SendACK;}
void setRampStart(void)       { prepairRamp(); executeRamp(); SendACK; }
void setRampTrigger(char *val)
{
  String token = val;
  if(token == "OFF")           rfgendata.rampTriggerOptions = RAMP_TRIG_OFF;
  else if(token == "ACTIVE")   rfgendata.rampTriggerOptions = RAMP_TRIG_ACTIVE;
  else if(token == "INACTIVE") rfgendata.rampTriggerOptions = RAMP_TRIG_INACTIVE;
  else BADARG;
  SendACK;
}
void getRampTrigger(void)
{
  SendACKonly;
  if(rfgendata.rampTriggerOptions == RAMP_TRIG_OFF)      serial->println("OFF");
  if(rfgendata.rampTriggerOptions == RAMP_TRIG_ACTIVE)   serial->println("ACTIVE");
  if(rfgendata.rampTriggerOptions == RAMP_TRIG_INACTIVE) serial->println("INACTIVE");
}

void trig_isr(void)
{
  int pin = digitalRead(TRIG);
  digitalWrite(STATUS_LT,LOW);

  if(((pin == LOW) && (rfgendata.triggerLevel == ACTIVE_HIGH)) || ((pin == HIGH) && (rfgendata.triggerLevel == ACTIVE_LOW)))
  {
    if(rfgendata.rampTriggerOptions == RAMP_TRIG_ACTIVE)
    {
      prepairRamp();
      executeRamp();
    }
    else currentBias = rfgendata.Bias;
    writeBIAS;
    digitalWrite(RFDISABLE,LOW);
  }
  else
  {
    if(rfgendata.rampTriggerOptions == RAMP_TRIG_INACTIVE)
    {
      prepairRamp();
      executeRamp();
    }
    else currentBias = 0;
    digitalWrite(RFDISABLE,HIGH);
    writeBIAS;
  }
  digitalWrite(STATUS_LT,HIGH);
}
// External trigger options, OFF | GATE
// In GATE mode: 
//              - When gate is active BIAS is set to user defined value
//              - When gate is inactive BIAS is set to 0
void setTrigger(char *val) 
{
  String token = val;
  if(token == "OFF") 
  {
    rfgendata.tiggerOptions = TRIG_OFF;
    detachInterrupt(digitalPinToInterrupt(TRIG));
    digitalWrite(RFDISABLE,LOW);
    currentBias = rfgendata.Bias;
  }
  else if(token == "GATE") 
  {
    rfgendata.tiggerOptions = TRIG_GATE;
    attachInterrupt(digitalPinToInterrupt(TRIG), trig_isr, CHANGE);
  }
  else BADARG;
  SendACK;
}

void getTrigger(void) 
{
  SendACKonly;
  if(rfgendata.tiggerOptions == TRIG_OFF)  serial->println("OFF");
  if(rfgendata.tiggerOptions == TRIG_GATE) serial->println("GATE");
}
void setTriggerLevel(char *val) 
{
  String token = val;
  if(token == "HIGH")     rfgendata.triggerLevel = ACTIVE_HIGH;
  else if(token == "LOW") rfgendata.triggerLevel = ACTIVE_LOW;
  else BADARG;
  SendACK;
}
void getTriggerLevel(void) 
{
  SendACKonly;
  if(rfgendata.triggerLevel == ACTIVE_HIGH) serial->println("HIGH");
  if(rfgendata.triggerLevel == ACTIVE_LOW)  serial->println("LOW");
}
#endif
