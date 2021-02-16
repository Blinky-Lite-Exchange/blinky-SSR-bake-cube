#include <SPI.h> 
#define CS0Pin 10
#define BC547Pin 17
#define blueLedPin 15
#define redLedPin 16

boolean blueLed = false; 
boolean redLed = true; 
boolean bC547 = false; 
SPISettings spiSettings(16000000, MSBFIRST, SPI_MODE1); 

void setup() 
{
  pinMode (blueLedPin, OUTPUT);
  pinMode (redLedPin, OUTPUT);
  pinMode (CS0Pin, OUTPUT);
  pinMode (BC547Pin, OUTPUT);
  digitalWrite(blueLedPin,blueLed);
  digitalWrite(redLedPin,redLed);
  digitalWrite(CS0Pin,1);
  digitalWrite(BC547Pin,bC547);
  Serial.begin(9600);
  SPI.begin();
}

void loop() 
{
  float fTemp1 = 0.0;
  unsigned long startTime = micros();
  fTemp1 = readTemp(CS0Pin);
  Serial.println(micros() - startTime);

  
  Serial.println(fTemp1);
  delay(2000);  
  digitalWrite(BC547Pin,bC547);
  bC547 = !bC547;
  digitalWrite(blueLedPin,blueLed);
  blueLed = !blueLed;
  digitalWrite(redLedPin,redLed);
  redLed = !redLed;

}

float readTemp(int chipSelect)
{
  uint8_t  dataBufRead[4];
  int bits[32];
  int iTemp = 0;
  float fTemp;
  int pow2 = 1; 

  SPI.beginTransaction(spiSettings);
  digitalWrite (chipSelect, LOW);
  SPI.transfer(&dataBufRead, 4);
  digitalWrite (chipSelect, HIGH);
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
