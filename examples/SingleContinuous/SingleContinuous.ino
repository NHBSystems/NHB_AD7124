/*
  AD7124 Single channel example
  
  Reads a load cell on channel 0 in continuous conversion mode. This example 
  sets output data rate to 1200 sps, but you can change the value of
  filterSelectBits to try other rates. The maximum sample rate will also be 
  dependant on what microcontroller you are using, and it's maximum serial baud 
  rate. Of course doing serial output in the sampling loop creates a huge 
  bottleneck, but I think it makes for a cleaner and more usefull example.

  In full power mode, the output rate for a single channel can be calculated
  with the following equation: 
  
  rate = 614,400/(32 x filterSelectBits)

  filterSelectBits can be any value between 1 and 2047, with 1 being the
  fastest output (19,200) and 2047 the slowest (9.38)


                                *  *  *  *
  -On a Feather M0, 1200 sps (fs = 16) is the fastest this sketch will run stably
  
  -On an ESP32 Feather, I could get a stable 6400 sps (fs = 3) with the baud 
   rate set to 1000000.

  -With a Teensy 3.2, this sketch can run at 9600 sps (fs = 2) I think this is
   possible because the Teensy 3.2 runs serial at full 12Mbit/sec USB speed. 
   
   At these rates the Arduino serial monitor kind of locks up after a few 
   seconds anyway. Something like CoolTerm will work fine though.
  
                                 *  *  *  *  
  
  NOTE: Continuous conversion mode may not play nice with other devices on the 
  SPI bus, especially if running at high output rates.

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
// 1 = Minimum filter, Maximum sample rate
// 2047 = Maximum filter, Minumum sample rate 
// 16 = 1200 sps
uint16_t filterSelectBits = 16;  


void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only

  Serial.println ("AD7124 1 channel example");


  // Initializes the AD7124 device
  adc.begin();
  
  // Configuring ADC for continuous conversion in full power mode (Fastest)
  adc.setAdcControl(AD7124_OpMode_Continuous, AD7124_FullPower, true);

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


// Take readings, and calculate the sampling rate over 1 second intervals. 
// You can change the filterSelectBits above and see how it affects the 
// output rate and the noise in your signal. 
void loop() {
  double reading;
  
  uint32_t count = 0;
  static uint32_t lastCount = 0;
  uint32_t interval = millis() + 1000;
 

  while(millis() < interval)
  { 
    reading = adc.readFB(0, 2.5, 5.00);
    
    // Doing serial output in the sampling loop creates a bottleneck, but 
    // I think it makes for a cleaner and more usefull example.
    Serial.print(reading);
    Serial.print('\t');
    Serial.print(lastCount);
    Serial.println("sps");

    count++;
  }

  lastCount = count;

}
