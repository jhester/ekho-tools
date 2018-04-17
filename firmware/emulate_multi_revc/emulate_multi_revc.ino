#include <Wire.h>
#include <ADC.h>
#include <ADC_Module.h>
#include <RingBuffer.h>
#include <RingBufferDMA.h>
#include <SPI.h>

#define AD5282_ADDRESS 0b0101100
#define AD5282_RDAC1 0b00000000
#define AD5282_RDAC2 0b10000000
#define DATA_POINTS_N 65

#define NUM_CURVES 7

#define DAC_PIN A22
#define VS_ADC_PIN A1
#define STAT_PIN 23
#define CS_PIN 33


ADC *adc = new ADC();

elapsedMillis timeElapsed;
uint16_t * current_curve;
uint8_t curve_index = 0;
uint8_t pin_state = 0;


uint16_t default_curve[NUM_CURVES][DATA_POINTS_N] = {125,160,194,230,266,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,301,
230,259,288,317,346,375,403,433,462,491,519,546,573,598,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,
148,184,219,255,291,326,362,397,433,469,504,540,576,611,647,683,718,754,790,825,860,895,929,963,996,1029,1062,1094,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,1127,
741,752,763,775,786,797,808,819,830,841,852,864,875,886,897,908,919,930,942,953,964,975,986,997,1008,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,
1256,1265,1274,1283,1292,1301,1310,1318,1327,1336,1345,1354,1363,1372,1381,1390,1398,1407,1416,1425,1434,1443,1452,1461,1470,1479,1487,1496,1505,1514,1523,1532,1541,1550,1559,1568,1576,1585,1594,1603,1612,1621,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,1630,
741,752,763,775,786,797,808,819,830,841,852,864,875,886,897,908,919,930,942,953,964,975,986,997,1008,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,1019,
230,259,288,317,346,375,403,433,462,491,519,546,573,598,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624,624};

uint16_t off_curve[DATA_POINTS_N] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16_t voltage=0;
unsigned char buf[512];
uint32_t i = 0;
uint8_t z = 0;


inline uint16_t interpolate_f(uint16_t _adc_14_value, uint16_t _shift_bits) 
{
  uint16_t left = _adc_14_value >> _shift_bits;
  return  current_curve[left] -
        ((
          (uint32_t)(current_curve[left] - current_curve[left+1]) * 
          (uint32_t)(_adc_14_value - (left << _shift_bits))
        ) >> _shift_bits);
};

void set_rdac(uint8_t num, uint8_t val) {
  Wire.beginTransmission(AD5282_ADDRESS);
  Wire.write(num);            
  Wire.write(val);             
  Wire.endTransmission();     
}

void reset_digpot() {
  set_rdac(AD5282_RDAC1, 255);
  set_rdac(AD5282_RDAC2, 255);
}

void setup() {
  Serial.begin(115200);
  
  pinMode(STAT_PIN, OUTPUT);
  digitalWrite(STAT_PIN, LOW);

 // Make sure higest value on Digipot so it does not go that path
  Wire.begin(); // join i2c bus (address optional for master)
  delay(100);
  reset_digpot();
  
  ////// ADC0 /////
  adc->setAveraging(1); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed 
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

  // 12-bit DAC
  analogWriteResolution(12); 
  analogWrite(DAC_PIN, 1);

  // Let everything settle
  current_curve = default_curve[0];
  delay(1000);
  timeElapsed = 0;
}

void loop() {
   //digitalWrite(STAT_PIN, LOW);
   for(i=0;i<1000;i++) {
    voltage = adc->analogRead(VS_ADC_PIN, ADC_0);
    analogWrite(DAC_PIN, interpolate_f(voltage, 6) );
   }
   if(timeElapsed > 1000) { 
    timeElapsed = 0;
    uint8_t next_curve = (curve_index++) % NUM_CURVES;
    current_curve = default_curve[next_curve];
    digitalWrite(STAT_PIN, pin_state);
    pin_state = !pin_state;
   }
   
}



