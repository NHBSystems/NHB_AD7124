/*
  AD7124 Single channel example
  
  Reads a load cell on channel 0 in single conversion mode.  
  
  Uses the switched 2.5V excitation provided on the NHB AD7124 board, which must
  be turned on before reading the sensor. On NHB boards the on chip low side switch
  is tied to the enable pin of a 2.5V regulator to provide excitation voltage to
  sensors. This allows the regulator to be shut down between readings to save power
  when doing long term, low speed logging.
  
  For more on AD7124, see
  http://www.analog.com/media/en/technical-documentation/data-sheets/AD7124-4.pdf

  This file is part of the NHB_AD7124 library.

  MIT License - A copy of the full text should be included with the library

  Copyright (C) 2021  Jaimy Juliano

*/

#include <NHB_AD7124.h>


const uint8_t csPin = 10;

Ad7124 adc(csPin, 4000000);



// The filter select bits determine the filtering and ouput data rate
//
//     1 = Minimum filter, fastest conversion time, maximum output rate.
//  2047 = Maximum filter, slowest conversion time, minumum output rate.
//
// For this example I'll use a setting of 320 with the Sinc3 filter. This 
// should provide 95dB of 60 Hz rejection and will output at around 20 sps.
//
// Of course you can always take your readings at a SLOWER rate than
// the output data rate. (i.e. logging a reading every  30 seconds)
//
// NOTE: Actual output data rates in single conversion mode will be slower
// than calculated using the formula in the datasheet. This is because of
// the settling time plus the time it takes to enable or change the channel.
uint16_t filterSelectBits = 320;


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

  // Set filter type and data rate select bits (defined above)
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelectBits);

  // Setting channel 0 using pins AIN1(+)/AIN0(-)
  adc.setChannel(0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true);

  // Turn on excitation voltage. 
  adc.setPWRSW(1);
}


void loop() {
  double reading; 

  uint32_t count = 0;
  static uint32_t lastCount = 0;
  uint32_t interval = millis() + 1000;
 

  while(millis() < interval)
  { 
    reading = adc.readFB(0, 2.5, 5.00);
    
    
    Serial.print(reading,4);
    Serial.print('\t');
    Serial.print(lastCount);
    Serial.println("sps");

    count++;
  }

  lastCount = count; 
  
}
