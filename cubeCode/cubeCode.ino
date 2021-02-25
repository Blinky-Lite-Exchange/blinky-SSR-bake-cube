#include <SPI.h> 
#define BAUD_RATE 115200
#define CS0Pin 10
#define ssrOnPin 17
#define ssrOnLed 16
#define CHECKSUM 36
#define LINE_PERIOD 20

struct TransmitData
{
  float temp = -100.0;
  float dutyFactor = 0;
  float avgDutyFactor = 0.0;
  int checkSum = CHECKSUM;
};
struct ReceiveData
{
  int regState = 0;
  float num50HzCycles = 100.0;
  float targetTemp = 20.0;
  float propK = 0.01;
  float intK = 100.0;
  float dutyFactorSet = 0.0;
  float numTempSamples = 1.0;
  int checkSum = CHECKSUM;
};
SPISettings spiSettings(16000000, MSBFIRST, SPI_MODE1); 
boolean resetTempMeas = true;

void setupPins()
{
  pinMode (CS0Pin, OUTPUT);
  pinMode (ssrOnPin, OUTPUT);
  pinMode (ssrOnLed, OUTPUT);
  digitalWrite (ssrOnPin, LOW);
  digitalWrite (ssrOnLed, LOW);
  digitalWrite (CS0Pin, HIGH);
  resetTempMeas = true;

  SPI.begin();
}
void processNewSetting(TransmitData* tData, ReceiveData* rData, ReceiveData* newData)
{
  rData->checkSum       = newData->checkSum;
  rData->propK          = newData->propK;
  rData->targetTemp     = newData->targetTemp;
  rData->intK           = newData->intK;
  rData->num50HzCycles  = newData->num50HzCycles;
  rData->dutyFactorSet  = newData->dutyFactorSet;
  if (newData->regState != rData->regState)
  {
    tData->avgDutyFactor = 0.0;
    rData->regState      = newData->regState;
  }
  if (newData->numTempSamples != rData->numTempSamples)
  {
    resetTempMeas = true;
    rData->numTempSamples      = newData->numTempSamples;
  }
}
boolean processData(TransmitData* tData, ReceiveData* rData)
{
  float errorTemp;
  float milliOn;
  float milliOff;
  float sampleTemp = 0.0;
  
  sampleTemp = readTemp();
  if (resetTempMeas)
  {
    resetTempMeas = false;
    tData->temp = sampleTemp;
  }
  else
  {
    tData->temp = tData->temp + (sampleTemp - tData->temp) / rData->numTempSamples;
  }
  
  if (rData->regState == 0)
  {
    digitalWrite (ssrOnPin, LOW);
    digitalWrite (ssrOnLed, LOW);
    tData->avgDutyFactor = 0.0;
    tData->dutyFactor = 0.0;
  }
  if (rData->regState == 1)
  {
    errorTemp = rData->targetTemp - tData->temp;
    tData->avgDutyFactor = tData->avgDutyFactor + 0.001 * rData->intK * rData->num50HzCycles * LINE_PERIOD * errorTemp / 6000.0;
    tData->dutyFactor = 0.01 * rData->propK * errorTemp + tData->avgDutyFactor;
    if (tData->dutyFactor < 0.0) tData->dutyFactor = 0.0;
    if (tData->dutyFactor > 1.0) tData->dutyFactor = 1.0;
  }
  if (rData->regState == 2)
  {
    digitalWrite (ssrOnPin, HIGH);
    digitalWrite (ssrOnLed, HIGH);
    tData->dutyFactor = rData->dutyFactorSet * 0.01;
    tData->avgDutyFactor = tData->dutyFactor;
  }
  milliOn = tData->dutyFactor * LINE_PERIOD * rData->num50HzCycles;
  milliOff = (1.0 - tData->dutyFactor) * LINE_PERIOD * rData->num50HzCycles;
  if (milliOn > (LINE_PERIOD * 0.2))
  {
    digitalWrite (ssrOnPin, HIGH);
    digitalWrite (ssrOnLed, HIGH);
  }
  delay((unsigned long) milliOn);
  digitalWrite (ssrOnPin, LOW);
  digitalWrite (ssrOnLed, LOW);
  delay((unsigned long) milliOff);
  return true;
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
const int commLEDPin = 15;
boolean commLED = true;

struct TXinfo
{
  int cubeInit = 1;
  int newSettingDone = 0;
};
struct RXinfo
{
  int newSetting = 0;
};

struct TX
{
  TXinfo txInfo;
  TransmitData txData;
};
struct RX
{
  RXinfo rxInfo;
  ReceiveData rxData;
};
TX tx;
RX rx;
ReceiveData settingsStorage;

int sizeOfTx = 0;
int sizeOfRx = 0;

void setup()
{
  setupPins();
  pinMode(commLEDPin, OUTPUT);  
  digitalWrite(commLEDPin, commLED);

  sizeOfTx = sizeof(tx);
  sizeOfRx = sizeof(rx);
  Serial1.begin(BAUD_RATE);
  delay(1000);
}
void loop()
{
  boolean goodData = false;
  goodData = processData(&(tx.txData), &settingsStorage);
  if (goodData)
  {
    tx.txInfo.newSettingDone = 0;
    if(Serial1.available() > 0)
    { 
      commLED = !commLED;
      digitalWrite(commLEDPin, commLED);
      Serial1.readBytes((uint8_t*)&rx, sizeOfRx);
      
      if (rx.rxInfo.newSetting > 0)
      {
        processNewSetting(&(tx.txData), &settingsStorage, &(rx.rxData));
        tx.txInfo.newSettingDone = 1;
        tx.txInfo.cubeInit = 0;
      }
    }
    Serial1.write((uint8_t*)&tx, sizeOfTx);
  }
  
}
