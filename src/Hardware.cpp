#include <Arduino.h>
#include "Hardware.h"
#include "Serial.h"
#include "AtomicBlock.h"

Si5351 si5351;

// Reads the selected ADC channel for the number of averages defined by num
int GetADCvalue(int chan, int num)
{
  int i=0,j;

  for(j=0;j<num;j++) i += analogRead(chan);
  return i/num;
}

// Counts to value and value to count conversion functions.
// Overloaded for both DACchan and ADCchan structs.
float Counts2Value(int Counts, DACchan *DC)
{
  return (Counts - DC->b) / DC->m;
}

float Counts2Value(int Counts, ADCchan *ad)
{
  return (Counts - ad->b) / ad->m;
}

int Value2Counts(float Value, DACchan *DC)
{
  int counts;

  counts = (Value * DC->m) + DC->b;
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

int Value2Counts(float Value, ADCchan *ac)
{
  int counts;

  counts = (int)((Value * ac->m) + ac->b);
  if (counts < 0) counts = 0;
  if (counts > 65535) counts = 65535;
  return (counts);
}

// This function reads and returns an ADC channel value. The raw ADC
// value is read and converted to engineering units.
float ReadADCchannel(ADCchan adch)
{
  int adc = GetADCvalue(adch.Chan,20);
  return Counts2Value(adc,&adch);
}

// Adafruit Itsybity M4 Only: Set-up digital pin D7 to 50KHz PWM output
// and set to near 100% duty cycle 
void setupD10pwm()
{
  // Set up the generic clock (GCLK7) to clock timer TCC0
  GCLK->GENCTRL[7].reg = GCLK_GENCTRL_DIV(4) |       // Divide the 48MHz clock source by divisor 3: 100MHz/2 = 50MHz
                         GCLK_GENCTRL_IDC |          // Set the duty cycle to 50/50 HIGH/LOW
                         GCLK_GENCTRL_GENEN |        // Enable GCLK7
                         //GCLK_GENCTRL_SRC_DFLL;      // Select 48MHz DFLL clock source
                         GCLK_GENCTRL_SRC_DPLL0;     // Select 100MHz DPLL clock source
                         //GCLK_GENCTRL_SRC_DPLL0;     // Select 120MHz DPLL clock source
  while (GCLK->SYNCBUSY.bit.GENCTRL7);               // Wait for synchronization 

  GCLK->PCHCTRL[25].reg = GCLK_PCHCTRL_CHEN |        // Enable the TCC0 perhipheral channel
                          GCLK_PCHCTRL_GEN_GCLK7;    // Connect generic clock 7 to TCC0

  // Enable the peripheral multiplexer on pin D10
  PORT->Group[g_APinDescription[10].ulPort].PINCFG[g_APinDescription[10].ulPin].bit.PMUXEN = 1;
 
  // Set the D10 (PORT_PA20) peripheral multiplexer to peripheral (even port number) E(6): TCC0, Channel 0, 6 = group G
  PORT->Group[g_APinDescription[10].ulPort].PMUX[g_APinDescription[10].ulPin >> 1].reg |= PORT_PMUX_PMUXE(6);
 
  TCC0->CTRLA.reg = TC_CTRLA_PRESCALER_DIV1 |        // Set prescaler to 16, 50MHz/1 = 50MHz
                    TC_CTRLA_PRESCSYNC_PRESC;        // Set the reset/reload to trigger on prescaler clock                 

  TCC0->WAVE.reg = TC_WAVE_WAVEGEN_NPWM;             // Set-up TCC0 timer for Normal (single slope) PWM mode (NPWM)
  while (TCC0->SYNCBUSY.bit.WAVE)                    // Wait for synchronization
 
//  TCC0->PER.reg = 999;                               // Set-up the PER (period) register 50KHz PWM
  TCC0->PER.reg = 600;                               // Set-up the PER (period) register 50KHz PWM
  while (TCC0->SYNCBUSY.bit.PER);                    // Wait for synchronization
 
  TCC0->CC[0].reg = 740;                             // close to 100%
  while (TCC0->SYNCBUSY.bit.CC0);                    // Wait for synchronization

  TCC0->CTRLA.bit.ENABLE = 1;                        // Enable timer TCC0
  while (TCC0->SYNCBUSY.bit.ENABLE);                 // Wait for synchronization
}

// Set PWM to 0 to 999
void setD10pwm(int value)
{ 
  if(value < 0) value = 0;
  if(value > 999) value = 999;
  TCC0->CCBUF[0].reg = value;
}

bool si5351initPLL(void)
{
  bool i2c_found;

  i2c_found = si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);
  if(!i2c_found) return false;

  si5351.output_enable(SI5351_CLK1,false);
  si5351.output_enable(SI5351_CLK2,false);
  // Set CLK0 to output 9 MHz
  si5351.set_freq(900000000ULL, SI5351_CLK0);

  // Query a status update and wait a bit to let the Si5351 populate the
  // status flags correctly.
  si5351.update_status();
  delay(500);
  return true;
}

void si5351setFrequency(int freq)
{
  si5351.set_freq(freq * 100, SI5351_CLK0);
}

// Set frequency, 1000000 to 5000000.
// Any out of range value will turn off the RF
void setFrequency(int value)
{
  static int init = false;

  if((value < 400000) || (value > 5000000))
  {
    // Turn off the RF signal to FET and exit
    return;
  }
  if(!init)
  {
    #ifdef RFGEN2
    si5351initPLL();
    #endif
    init = true;
  }
  #ifdef RFGEN2
  si5351setFrequency(value);
  #else
  FS7140setup(rfgendata.TWIadd,value);
  #endif
}

// Set drive to 0 to 100%
void setDrive(float percent)
{
  int value;
  
  if(percent < 0.0)   percent = 0.0;
  if(percent > 100.0) percent = 100.0;
  percent = 100 - percent;
  value = (MAXDRIVE * (100.0 - percent)) / 100.0;
  if(value >= MAXDRIVE)
  {
    value = MAXDRIVE - 1;
    setD10pwm(value);
    return;
  }
  if(value < 0) value = 0;
  setD10pwm(value);
}

void ComputeCRCbyte(byte *crc, byte by)
{
  byte generator = 0x1D;

  *crc ^= by;
  for(int j=0; j<8; j++)
  {
    if((*crc & 0x80) != 0)
    {
      *crc = ((*crc << 1) ^ generator);
    }
    else
    {
      *crc <<= 1;
    }
  }
}

// Compute 8 bit CRC of buffer
byte ComputeCRC(byte *buf, int bsize)
{
  byte generator = 0x1D;
  byte crc = 0;

  for(int i=0; i<bsize; i++)
  {
    crc ^= buf[i];
    for(int j=0; j<8; j++)
    {
      if((crc & 0x80) != 0)
      {
        crc = ((crc << 1) ^ generator);
      }
      else
      {
        crc <<= 1;
      }
    }
  }
  return crc;
}

// The function will program the FLASH memory by receiving a file from the USB connected host. 
// The file must be sent in hex and use the following format:
// First the FLASH address in hex and file size, in bytes (decimal) are sent. If the file can
// be burned to FLASH an ACK is sent to the host otherwise a NAK is sent. The process stops
// if a NAK is sent. 
// If an ACK is sent to the host then the host will send the data for the body of the 
// file in hex. After all the data is sent then a 8 bit CRC is sent, in decimal. If the
// crc is correct and ACK is returned.
void ProgramFLASH(char * Faddress,char *Fsize)
{
  static String sToken;
  static uint32_t FlashAddress;
  static int    numBytes,fi,val,tcrc;
  static char   c,buf[3],*Token;
  static byte   fbuf[256],b,crc;
  static byte   vbuf[256];
  static uint32_t start;

  crc = 0;
  FlashAddress = strtol(Faddress, 0, 16);
  sToken = Fsize;
  numBytes = sToken.toInt();
  SendACK;
  fi = 0;
  FlashClass fc((void *)FlashAddress,numBytes);
  for(int i=0; i<numBytes; i++)
  {
    start = millis();
    // Get two bytes from input ring buffer and scan to byte
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[0] = c;
    while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    buf[1] = c;
    buf[2] = 0;
    sscanf(buf,"%x",&val);
    fbuf[fi++] = val;
    ComputeCRCbyte(&crc,val);
    if(fi == 256)
    {
      fi = 0;
      // Write the block to FLASH
      noInterrupts();
      fc.erase((void *)FlashAddress, 256);
      fc.write((void *)FlashAddress, fbuf, 256);
      // Read back and verify
      fc.read((void *)FlashAddress, vbuf, 256);
      for(int j=0; j<256; j++)
      {
        if(fbuf[j] != vbuf[j])
        {
           interrupts();
           serial->println("FLASH data write error!");
           SendNAK;
           return;   
        }
      }
      interrupts();
      FlashAddress += 256;
      serial->println("Next");
    }
  }
  // If fi is > 0 then write the last partial block to FLASH
  if(fi > 0)
  {
    noInterrupts();
    fc.erase((void *)FlashAddress, fi);
    fc.write((void *)FlashAddress, fbuf, fi);
    // Read back and verify
    fc.read((void *)FlashAddress, vbuf, fi);
    for(int j=0; j<fi; j++)
    {
      if(fbuf[j] != vbuf[j])
      {
         interrupts();
         serial->println("FLASH data write error!");
         SendNAK;
         return;   
      }
    }
    interrupts();
  }
  // Now we should see an EOL, \n
  start = millis();
  while((c = RB_Get(&RB)) == 0xFF) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
  if(c == '\n')
  {
    // Get CRC and test, if ok exit else delete file and exit
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    sscanf(Token,"%d",&tcrc);
    while((Token = GetToken(true)) == NULL) { ProcessSerial(false); if(millis() > start + 10000) goto TimeoutExit; }
    if((Token[0] == '\n') && (crc == tcrc)) 
    {
       serial->println("File received from host and written to FLASH.");
       SendACK;
       return;
    }
  }
  serial->println("\nError during file receive from host!");
  SendNAK;
  return;
TimeoutExit:
  serial->println("\nFile receive from host timedout!");
  SendNAK;
  return;
}

// These functions support continous DMA transfer from the variable constantDacValue to
// The DAC. This code is designed for the Adafruit itsybitsy M4 ARM processor. Configure
// the DAC resolution before initilizing this feture.
// Use the following call order:
//  dma_init(0);    // DMA channel nnumber
//  dac_init(0,6);  // DAC channel and clock channel. Note the dma_init function will need
//                  // to be edited for DAC channel 1.


// The single constant value to output via DAC
// This value should be within the DAC's range (0-1023 for 10-bit, 0-4095 for 12-bit)
uint16_t volatile constantDacValue = 1500;

// Define the total number of DMA channels your SAMD51 chip has
// (e.g., 16 for SAMD51). You can find this in your chip's datasheet or header.
#define DMAC_NUM_CHANNELS 16 // Example: SAMD51 has 16 DMA channels

// DMA Descriptors - must be 16-byte aligned
static DmacDescriptor descriptor1[DMAC_NUM_CHANNELS] __attribute__((aligned(16)));
static DmacDescriptor wrb[DMAC_NUM_CHANNELS] __attribute__((aligned(16))); // Write-back descriptor

// --- DMA Initialization Function ---
// The dma_init function initializes the Direct Memory Access (DMA) controller for a specified 
// channel, configuring the DMA settings, including the base address, trigger source, and 
// descriptor for data transfer. It ensures that the DMA is enabled and ready for burst 
// transfers from a constant value to the DAC data register.
void dma_init(int dmach) 
{
  MCLK->AHBMASK.bit.DMAC_ = 1;
  
  DMAC->CTRL.bit.DMAENABLE = 0;
  DMAC->CTRL.bit.SWRST = 1;
  // 1. Configure the DMAC (DMA Controller)
  DMAC->BASEADDR.reg = (uint32_t)&descriptor1[0];  // Base address of the primary descriptor
  DMAC->WRBADDR.reg  = (uint32_t)&wrb[0];          // Write-back address register

  // Enable DMA transfer levels (0-3) - ensures channels can operate
  DMAC->CTRL.bit.LVLEN0 = 1;
  DMAC->CTRL.bit.LVLEN1 = 1;
  DMAC->CTRL.bit.LVLEN2 = 1;
  DMAC->CTRL.bit.LVLEN3 = 1;
  DMAC->CTRL.bit.DMAENABLE = 1; // Enable the entire DMAC

  DMAC->Channel[dmach].CHCTRLA.bit.ENABLE = 0;
  DMAC->Channel[dmach].CHCTRLA.bit.SWRST = 1;
  while(DMAC->Channel[dmach].CHCTRLA.bit.SWRST);

  DMAC->Channel[dmach].CHPRILVL.reg = 1;

  // 2. Configure DMA Channel 0
  // Trigger Source: DAC Data Register (0x48 - 0x49 on SAMD51)
  // 0x48 for DAC channel 0
  // 0x49 for DAC channel 1
  // Trigger Action: Burst transfers (each trigger initiates one beat)
  DMAC->Channel[dmach].CHCTRLA.reg = DMAC_CHCTRLA_TRIGSRC(0x48) | DMAC_CHCTRLA_TRIGACT_BURST;
  DMAC->Channel[dmach].CHCTRLA.bit.BURSTLEN = 1;

  // 3. Configure the DMA Descriptor (descriptor1)
  // Self-referencing descriptor for continuous looping:
  // After this block transfer, the DMAC will reload this same descriptor
  descriptor1[dmach].DESCADDR.reg = (uint32_t) &descriptor1[dmach];
  descriptor1[dmach].BTCTRL.bit.VALID    = 0x1; // Mark as a valid descriptor
  descriptor1[dmach].BTCTRL.bit.BEATSIZE = 0x1; // Beat size is Half-Word (16-bit), matching uint16_t
  descriptor1[dmach].BTCTRL.bit.SRCINC   = 0x0; // Source Increment Disabled (always read from 'constantDacValue')
  descriptor1[dmach].BTCTRL.bit.DSTINC   = 0x0; // Destination Increment Disabled (always write to DAC_DATA[0])
  descriptor1[dmach].BTCTRL.bit.BLOCKACT = 0x0; // No action (allows re-triggering)
  descriptor1[dmach].BTCTRL.bit.STEPSIZE = 1;

  descriptor1[dmach].BTCNT.reg           = 1; // Block Transfer Count: Transfer only 1 beat (the single value)
  descriptor1[dmach].SRCADDR.reg         = (uint32_t)&constantDacValue; // Source Address: The address of our constant value
  descriptor1[dmach].DSTADDR.reg         = (uint32_t)&DAC->DATA[0].reg; // Destination Address: DAC Channel 0 Data Register

  // 4. Enable DMA Channel 0
  DMAC->Channel[dmach].CHCTRLA.bit.ENABLE = 0x1;
  DMAC->CTRL.bit.DMAENABLE = 1; // Redundant, already enabled above
}

// --- DAC Initialization Function ---
// The dac_init function initializes a Digital-to-Analog Converter (DAC) by configuring the generic 
// clock generator, connecting it to the DAC peripheral, enabling the DAC clock, performing a 
// software reset, and setting up the DAC channel with specific parameters. It also includes 
// steps to ensure the DAC is ready for operation and configures the output pin for the DAC.
void dac_init(int dacCH, int clkCH) 
{
  // 1. Configure Generic Clock Generator 7 (GCLK7) for DAC
  // Source: 48MHz DFLL (Digital Frequency Locked Loop)
  // Division: Divide by 4, resulting in 12MHz (48MHz / 4 = 12MHz, max for DAC)
  GCLK->GENCTRL[clkCH].reg = GCLK_GENCTRL_DIV(4) | GCLK_GENCTRL_GENEN | GCLK_GENCTRL_SRC_DFLL;
  // Wait for GCLK7 to synchronize
  //while (GCLK->SYNCBUSY.bit.GENCTRL7);
  while (GCLK->SYNCBUSY.reg & (1UL << (GCLK_SYNCBUSY_GENCTRL0_Pos + clkCH))); // Corrected SYNCBUSY access

  // 2. Connect GCLK7 to the DAC Peripheral Clock Channel (PCHCTRL)
  // DAC_GCLK_ID (42) is the PCHCTRL ID for the DAC.
  GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN = 0;
  while(GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN != 0);
  //GCLK->PCHCTRL[DAC_GCLK_ID].bit.GEN = GCLK_PCHCTRL_GEN_GCLK7;
  GCLK->PCHCTRL[DAC_GCLK_ID].bit.GEN = clkCH;
  GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN = 1;
  while(GCLK->PCHCTRL[DAC_GCLK_ID].bit.CHEN != 1);
  
  // 3. Enable the clock for the DAC peripheral in the Main Clock (MCLK) system
  MCLK->APBDMASK.bit.DAC_ = 1;

  // 4. Software Reset the DAC peripheral
  DAC->CTRLA.bit.SWRST = 1;
  while (DAC->CTRLA.bit.SWRST); // Wait for reset to complete

  // 5. Configure DAC Channel 0
  // DAC_DACCTRL_REFRESH(2): Sets refresh period (adjust as needed for your application)
  // DAC_DACCTRL_CCTRL_CC12M: Sets contact current control (often related to internal buffer/reference)
  // DAC_DACCTRL_ENABLE: Enables DAC Channel 0
  DAC->DACCTRL[dacCH].reg = DAC_DACCTRL_REFRESH(2) | DAC_DACCTRL_CCTRL_CC12M | DAC_DACCTRL_ENABLE;

//    DAC->EVCTRL.bit.EMPTYEO0 = 1;

  // DAC_DACCTRL_LEFTADJ; // This line was a syntax error, removed. If needed, integrate into DACCTRL[0].reg |= ...
  // For example, if you need left adjustment: DAC->DACCTRL[0].bit.LEFTADJ = 1;

  // 6. Enable the main DAC peripheral
  DAC->CTRLA.reg = DAC_CTRLA_ENABLE;
  while (DAC->SYNCBUSY.reg & (1UL << DAC_SYNCBUSY_ENABLE_Pos)); // Wait for DAC enable synchronization

  // 7. Wait for DAC Channel 0 to be ready
  if(dacCH == 0) while (!DAC->STATUS.bit.READY0); // Use READY0 for Channel 0
  if(dacCH == 1) while (!DAC->STATUS.bit.READY1); // Use READY0 for Channel 0

  // 8. Configure the output pin (PA02 for DAC0) for Peripheral Function B
  // This sets PA02 as an output (DIRSET) and enables its Peripheral Multiplexing (PMUXEN)
  // PMUX[1] handles pins 2 and 3 in Group 0. PMUXE is for the even pin (PA02).
  // Value 0x1 corresponds to Peripheral Function B for DAC output.
  // The following code is for DAC0, it is not needed.
  //PORT->Group[0].DIRSET.reg = (1 << 2);    // Set PA02 (bit 2) as output
  //PORT->Group[0].PINCFG[2].bit.PMUXEN = 1; // Enable PMUX for PA02
  //PORT->Group[0].PMUX[1].bit.PMUXE = 0x1;  // Set PA02 to Peripheral Function B (DAC output)
}
