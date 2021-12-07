/*
  AD7124 Multiple sensor type example

  Demonstrate how to read multiple sensors of different types using different 
  channel configurations. This example configures the AD7124 for 3 differential
  and 2 single ended channels. It also reads the internal temperature sensor
  and uses it for the cold junction reference temperature.

  Uses the switched 2.5V excitation provided on the NHB AD7124 board, which must
  be turned on before reading the sensor. On NHB boards the on chip low side switch
  is tied to the enable pin of a 2.5V regulator to provide excitation voltage to
  sensors. This allows the regulator to be shut down between readings to save power
  when doing long term, low speed logging.

  
  The channels are configured as follows: 

  Channel 0   Differnetial    Load Cell (or other full bridge)
  Channel 1   Differential    Thermocouple
  Channel 2   Differential    Thermocouple   
  Channel 3   Single Ended    Potentiometer
  Channel 4   Single Ended    Potentiometer
  Channel 5   Internal        IC Temperature  


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


#define CH_COUNT 6 // 3 differential channels + 2 single ended + internal temperature sensor

const uint8_t ssPin = 10;

Ad7124 adc(ssPin, 4000000);


 
// The filter select bits determine the filtering and ouput data rate
// 1 = Minimum filter, Maximum sample rate
// 2047 = Maximum filter, Minumum sample rate
// A setting of 13 gets us about 50 sps with this channel configuration
int filterSelBits = 13; 


void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }

  Serial.println ("AD7124 Multiple sensor type example");

  // Initializes the AD7124 device
  adc.begin();

  // Configuring ADC in Full Power Mode (Fastest) 
  adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true);

 
  // Set the "setup" configurations for different sensor types. There are 7 differnet "setups"
  // in the ADC that can be configured. There are 8 setups that can be configured. Each 
  // setup holds settings for the reference used, the gain setting, filter type, and rate

  adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true);    // Load Cell:           External reference tied to excitation, Gain = 128, Bipolar = True
  adc.setup[1].setConfig(AD7124_Ref_Internal, AD7124_Gain_32, true);    // Thermocouple:        Internal reference, Gain = 128, Bipolar = True  
  adc.setup[2].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_1, false);     // Ratiometric Voltage: External reference tied to excitation, Gain = 1, Bipolar = False
  adc.setup[3].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true);     // IC Temp sensor:      Internal reference, Gain = 1, Bipolar = True



  // Filter settings for each setup. In this example they are all the same so
  // this really could have just been done with a loop
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[1].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[2].setFilter(AD7124_Filter_SINC3, filterSelBits);
  adc.setup[3].setFilter(AD7124_Filter_SINC3, filterSelBits);
  


  // Set channels, i.e. what setup they use and what pins they are measuring on
  
  adc.setChannel(0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true); //Channel 0 - Load cell
  adc.setChannel(1, 1, AD7124_Input_AIN2, AD7124_Input_AIN3, true); //Channel 1 - Type K Thermocouple 1
  adc.setChannel(2, 1, AD7124_Input_AIN4, AD7124_Input_AIN5, true); //Channel 2 - Type K Thermocouple 2 
  adc.setChannel(3, 2, AD7124_Input_AIN6, AD7124_Input_AVSS, true); //Channel 3 - Single ended potentiometer 1  
  adc.setChannel(4, 2, AD7124_Input_AIN7, AD7124_Input_AVSS, true); //Channel 4 - Single ended potentiometer 2
  adc.setChannel(5, 3, AD7124_Input_TEMP, AD7124_Input_AVSS, true); //Channel 5 - ADC IC temperature


  // For thermocouples we have to set the voltage bias on the negative input pin 
  // for the channel, in this case it's AIN3 and AIN5
  adc.setVBias(AD7124_VBias_AIN3,true); 
  adc.setVBias(AD7124_VBias_AIN5,true);  


  // Turn on excitation voltage. If you were only taking readings periodically
  // you could turn this off between readings to save power.
  adc.setPWRSW(1);
}

// -----------------------------------------------------------------------------

void loop() {

  double readings[CH_COUNT];
  long dt;

  //Take readings, and measure how long it takes just for fun
  //NOTE: On some architectures micros() is not very accurate 
  dt = micros();

  double junctionTemp = adc.readIcTemp(5); 
  
  readings[0] = adc.readFB(0,2.5, 5.00); //5.00 scaling for 10 lb load cell I use for testing
  readings[1] = adc.readTC(1, junctionTemp, Type_K); 
  readings[2] = adc.readTC(2, junctionTemp, Type_K);
  readings[3] = adc.readVolts(3);
  readings[4] = adc.readVolts(4);

  dt = micros() - dt;



  // You would probably do something more interesting with the readings here
  // but we'll just print them out for our example.

  Serial.print(readings[0], DEC); //Load Cell
  Serial.print('\t');
  Serial.print(readings[1], DEC); //Thermocouple
  Serial.print('\t');
  Serial.print(readings[2], DEC); //Thermocouple
  Serial.print('\t');
  Serial.print(readings[3], DEC); //Potentiometer
  Serial.print('\t');
  Serial.print(readings[4], DEC); //Potentiometer
  Serial.print('\t');
  Serial.print(junctionTemp, DEC); //IC Temp
  Serial.print('\t');
  Serial.print("dt=");
  Serial.print(dt);
  Serial.print("uS");
  Serial.println();

}