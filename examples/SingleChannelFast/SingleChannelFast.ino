/*
  AD7124 Single channel example
  
  Read a load cell on channel 0 as fast as we can. Note this will most likely
  produce very noisy results.

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


const uint8_t ledPin = 13;
const uint8_t ssPin = 10;

Ad7124 adc(ssPin, 4000000);

// TODO: Figure out why update rates don't match datasheet
// The filter select bits also determine the ouput data rate
// 1 = Minimum filter, maximum sample rate
uint16_t filterSelectBits = 1;


void setup() {

  pinMode (ledPin, OUTPUT);
  digitalWrite (ledPin, 1); // clear the LED

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only

  Serial.println ("AD7124 1 channel example");



  // For this example I will simply print out the return from each function call
  // so we can see if anything goes wrong 

  // Initializes the AD7124 device
  Serial.print("begin() ");
  Serial.println(adc.begin());
  
  // Configuring ADC in Full Power Mode (Fastest)
  Serial.print("setAdcControl() ");
  Serial.println(adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true));

  // Setting configuration for Setup 0:
  // - use the external reference tied to the excitation voltage (2.5V reg)
  // - gain of 128 for a bipolar measurement of +/- 19.53 mv
  Serial.print("setConfig() ");    
  Serial.println(adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true));
  //Serial.println(adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true, AD7124_Burnout_Off, 2.50)); // With optional args

  // Set filter type and data rate select bits (defined above)
  Serial.print("setFilter() ");
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelectBits);

  // Setting channel 0 using pins AIN1(+)/AIN0(-)
  Serial.print("setChannel() ");
  Serial.println(adc.setChannel (0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true));

  // Turn on excitation voltage
  Serial.print("setPWRSW() ");
  Serial.println(adc.setPWRSW(1));
}


void loop() {
  double voltage;
  long dt;
  
  //digitalWrite (ledPin, 0); // Uncomment for blinky lights

  //Take a voltage reading, and measure how long it takes
  dt = micros();   
  voltage = adc.readVolts(0);     
  dt = micros() - dt;

  //digitalWrite (ledPin, 1); // Uncomment for blinky lights

  Serial.print(voltage, DEC);
  Serial.print('\t');
  Serial.print("dt=");
  Serial.print(dt);
  Serial.print("uS");
  Serial.println();
  
}
