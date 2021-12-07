/*
  AD7124 Single channel example
  
  Reads a load cell on channel 0 as fast as we can (in single conversion mode). 
  Note this will most likely produce noisy results. 
  
  Uses the switched 2.5V excitation provided on the NHB AD7124 board, which must
  be turned on before reading the sensor. On NHB boards the on chip low side switch
  is tied to the enable pin of a 2.5V regulator to provide excitation voltage to
  sensors. This allows the regulator to be shut down between readings to save power
  when doing long term, low speed logging.

  NOTE: Currently I can not get the expected data rates per the datasheet. I may
  just be missing something dumb. I don't really care if I get the highest speed,
  but I want the rates to be predictable based on what the datasheet says.

  For more on AD7124, see
  http://www.analog.com/media/en/technical-documentation/data-sheets/AD7124-4.pdf

  This file is part of the NHB_AD7124 library.

  MIT License - A copy of the full text should be included with the library

  Copyright (C) 2021  Jaimy Juliano

*/

#include <NHB_AD7124.h>


const uint8_t ssPin = 10;

Ad7124 adc(ssPin, 4000000);



// The filter select bits determine the filtering and ouput data rate
// 1 = Minimum filter, Maximum sample rate
// 2047 = Maximum filter, Minumum sample rate
uint16_t filterSelectBits = 1;


void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only

  Serial.println ("AD7124 1 channel example");


  // Initializes the AD7124 device
  adc.begin();
  
  // Configuring ADC in Full Power Mode (Fastest)
  adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true);

  // Setting configuration for Setup 0:
  // - use the external reference tied to the excitation voltage (2.5V reg)
  // - gain of 128 for a bipolar measurement of +/- 19.53 mv
  adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true);
  //adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true, AD7124_Burnout_Off, 2.50); // With optional args

  // Set filter type and data rate select bits (defined above)
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelectBits);

  // Setting channel 0 using pins AIN1(+)/AIN0(-)
  adc.setChannel (0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true);

  // Turn on excitation voltage. 
  adc.setPWRSW(1);
}


void loop() {
  double voltage;
  long dt;
  

  //Take readings, and measure how long it takes
  //NOTE: On some architectures micros() is not very accurate 
  dt = micros();   
  voltage = adc.readVolts(0);     
  dt = micros() - dt;


  Serial.print(voltage, DEC);
  Serial.print('\t');
  Serial.print("dt=");
  Serial.print(dt);
  Serial.print("uS");
  Serial.println();
  
}
