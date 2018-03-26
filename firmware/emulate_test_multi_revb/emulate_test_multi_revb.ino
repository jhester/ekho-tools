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

ADC *adc = new ADC();

elapsedMillis timeElapsed;
uint16_t * current_curve;
uint16_t default_curve[DATA_POINTS_N] = {191,243,295,346,398,450,500,554,606,658,708,756,803,849,896,960,1024,1088,1152,1216,1280,1344,1408,1472,1536,1600,1664,1728,1792,1856,1920,1984,2048,2112,2176,2240,2304,2368,2432,2496,2560,2625,2689,2753,2817,2881,2945,3009,3073,3137,3201,3265,3329,3393,3457,3521,3585,3649,3713,3777,3841,3905,3969,4033,4095};
uint16_t low_curve[DATA_POINTS_N] = {30,94,158,222,285,349,413,475,536,596,654,710,768,832,896,960,1024,1088,1152,1216,1280,1344,1408,1472,1536,1600,1664,1728,1792,1856,1920,1984,2048,2112,2176,2240,2304,2368,2432,2496,2560,2625,2689,2753,2817,2881,2945,3009,3073,3137,3201,3265,3329,3393,3457,3521,3585,3649,3713,3777,3841,3905,3969,4033,4095};
uint16_t off_curve[DATA_POINTS_N] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
int16_t voltage=0;
unsigned char buf[512];
uint32_t i = 0;
uint8_t z = 0;

#define DAC_PIN A22
#define VS_ADC_PIN A1
#define STAT_PIN 23
#define CS_PIN 33

inline uint16_t interpolate_f(uint16_t _adc_14_value, uint16_t _shift_bits) 
{
	uint16_t left = _adc_14_value >> _shift_bits;
	return	current_curve[left] -
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

void setPot(int level) {
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0);
  SPI.transfer(level);
  digitalWrite(CS_PIN, HIGH);
}


void setup() {
	Serial.begin(115200);
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  
  pinMode(STAT_PIN, OUTPUT);
  digitalWrite(STAT_PIN, LOW);

 // Make sure higest value on Digipot so it does not go that path
  Wire.begin(); // join i2c bus (address optional for master)
  delay(100);
  reset_digpot();
  setPot(127);
  
  ////// ADC0 /////
  adc->setAveraging(1); // set number of averages
  adc->setResolution(12); // set bits of resolution
  adc->setConversionSpeed(ADC_CONVERSION_SPEED::VERY_HIGH_SPEED); // change the conversion speed 
  adc->setSamplingSpeed(ADC_SAMPLING_SPEED::VERY_HIGH_SPEED); // change the sampling speed

	// 12-bit DAC
	analogWriteResolution(12); 
	analogWrite(DAC_PIN, 1);

	// Let everything settle
	//for (int index=0; index < DATA_POINTS_N; index++) 
    //   current_curve[index] = (uint16_t) 0;
    current_curve = default_curve;
	delay(1000);
}

void loop() {
   //digitalWrite(STAT_PIN, LOW);
   for(i=0;i<1000;i++) {
   voltage = adc->analogRead(VS_ADC_PIN, ADC_0);
   analogWrite(DAC_PIN, interpolate_f(voltage, 6) );
   }
   //digitalWrite(STAT_PIN, HIGH);
   setPot(z++); 
   if(z>127) z=0;
  /*
   for(i=0;i<4095;i++) {
    analogWrite(DAC_PIN, i);
    delay(1);
   }
   */
}



