#include "BlinkyBus.h"
#include <SPI.h> 
#define BAUD_RATE  19200
#define CS0Pin 10
#define ssrOnPin 17
#define ssrOnLed 16
#define LINE_PERIOD 20
#define commLEDPin    15

#define BLINKYBUSBUFSIZE  11
union BlinkyBusUnion
{
  struct
  {
    int16_t state;
    int16_t regState;
    int16_t num50HzCycles;
    int16_t targetTemp10;
    int16_t propK100;
    int16_t intK100;
    int16_t dutyFactorSet10;
    int16_t numTempSamples;
    int16_t temp10;
    int16_t dutyFactor10;
    int16_t avgDutyFactor10;
  };
  int16_t buffer[BLINKYBUSBUFSIZE];
} bb;
BlinkyBus blinkyBus(bb.buffer, BLINKYBUSBUFSIZE, Serial1, commLEDPin);

SPISettings spiSettings(16000000, MSBFIRST, SPI_MODE1); 
boolean resetTempMeas = true;
int ipoll = 0;
float ftemp = 0.0;
float favgDutyFactor = 0.0;
float fdutyFactor = 0.0;
unsigned long tstart;
unsigned long tnow;

void setup() 
{
  pinMode (CS0Pin, OUTPUT);
  pinMode (ssrOnPin, OUTPUT);
  pinMode (ssrOnLed, OUTPUT);
  digitalWrite (ssrOnPin, LOW);
  digitalWrite (ssrOnLed, LOW);
  digitalWrite (CS0Pin, HIGH);
  resetTempMeas = true;
  SPI.begin();
  bb.regState = 0;
  bb.num50HzCycles = 100;
  bb.targetTemp10 = 250;
  bb.propK100 = 200;
  bb.intK100 = 100;
  bb.dutyFactorSet10 = 50;
  bb.numTempSamples = 4;
  bb.dutyFactor10 = 0;
  bb.avgDutyFactor10 = 0;

  bb.state = 1; //init
  Serial1.begin(BAUD_RATE);
  blinkyBus.start();
}

void loop() 
{

  float errorTemp;
  float milliOn;
  float milliOff;
  float sampleTemp = 0.0;
  
  sampleTemp = readTemp();
  if (sampleTemp > 2000.0)
  {
    resetTempMeas = true;
    ftemp = sampleTemp;
    bb.regState = 0;
  }
  else
  {
    if (resetTempMeas)
    {
      resetTempMeas = false;
      ftemp = sampleTemp;
    }
    else
    {
      ftemp = ftemp + (sampleTemp - ftemp) / ((float) bb.numTempSamples);
    }
  }

  if (bb.regState == 0)
  {
    digitalWrite (ssrOnPin, LOW);
    digitalWrite (ssrOnLed, LOW);
    favgDutyFactor = 0;
    fdutyFactor = 0;
  }
  if (bb.regState == 1)
  {
    errorTemp = 0.1 * ((float) bb.targetTemp10) - ftemp;
    favgDutyFactor = favgDutyFactor + 0.001 * 0.01 * ((float) bb.intK100) * ((float) bb.num50HzCycles) * LINE_PERIOD * errorTemp / 6000.0;
    if (favgDutyFactor < 0.0) favgDutyFactor = 0.0;
    if (favgDutyFactor > 1.0) favgDutyFactor = 1.0;
    fdutyFactor = 0.01 * 0.01 * ((float) bb.propK100) * errorTemp + favgDutyFactor;
    if (fdutyFactor < 0.0) fdutyFactor = 0.0;
    if (fdutyFactor > 1.0) fdutyFactor = 1.0;
  }
  if (bb.regState == 2)
  {
    digitalWrite (ssrOnPin, HIGH);
    digitalWrite (ssrOnLed, HIGH);
    fdutyFactor =  0.1 * ((float) bb.dutyFactorSet10) * 0.01;
    favgDutyFactor = fdutyFactor;
  }
  bb.temp10 = (int16_t) (ftemp * 10);
  bb.avgDutyFactor10 = (int16_t) (favgDutyFactor * 10);
  bb.dutyFactor10 = (int16_t) (fdutyFactor * 10);
  milliOn = fdutyFactor * LINE_PERIOD * ((float) bb.num50HzCycles);
  milliOff = (1.0 - fdutyFactor) * LINE_PERIOD * ((float) bb.num50HzCycles);
  if (milliOn > (LINE_PERIOD * 0.2))
  {
    digitalWrite (ssrOnPin, HIGH);
    digitalWrite (ssrOnLed, HIGH);
  }

  tstart = millis();
  tnow = tstart;
  while ((tnow - tstart) < ((unsigned long) milliOn) )
  {
    ipoll = blinkyBus.poll();
    if (ipoll == 2)
    {
      if (blinkyBus.getLastWriteAddress() == 1) favgDutyFactor = 0;
      if (blinkyBus.getLastWriteAddress() == 7) resetTempMeas = true;
    }
    tnow = millis();
  }
  digitalWrite (ssrOnPin, LOW);
  digitalWrite (ssrOnLed, LOW);
  tstart = millis();
  tnow = tstart;
  while ((tnow - tstart) < ((unsigned long) milliOff) )
  {
    ipoll = blinkyBus.poll();
    if (ipoll == 2)
    {
      if (blinkyBus.getLastWriteAddress() == 1) favgDutyFactor = 0;
      if (blinkyBus.getLastWriteAddress() == 7) resetTempMeas = true;
    }
    tnow = millis();
  }  
}

float readTemp()
{
  uint8_t  dataBufRead[4];
  int bits[32];
  int iTemp = 0;
  float fTemp;
  int pow2 = 1; 

  SPI.beginTransaction(spiSettings);
  digitalWrite (CS0Pin, LOW);
  SPI.transfer(&dataBufRead, 4);
  digitalWrite (CS0Pin, HIGH);
  SPI.endTransaction();

  for (int ibyte = 0; ibyte < 4; ++ibyte)
  {
    for (int ibit = 0; ibit < 8; ++ibit)
    {
      bits[31 - (ibyte * 8 + 7 - ibit)] = ((dataBufRead[ibyte] >> ibit) % 2);
    }
  }
  iTemp = 0;
  pow2 = 1;
  for (int ibit = 18; ibit < 31; ++ibit)
  {
    iTemp = iTemp + pow2 * bits[ibit];
    pow2 = pow2 * 2;
  }
  if (bits[31] > 0)
  {
    iTemp = iTemp - 8192;
  }
  fTemp = ((float) iTemp) * 0.25;
  return fTemp;
}
