/*
  AD7124 Multiple sensor type example

  Demonstrate how to read multiple sensors of different types using different 
  channel configurations. This example configures the AD7124 for 3 differential
  and 2 single ended channels. It also reads the internal temperature sensor
  and uses it for the cold junction reference temperature.

  
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

const uint8_t ledPin = 13;
const uint8_t ssPin = 10;

Ad7124 adc(ssPin, 4000000);

// TODO: Figure out why update rates don't match datasheet
// The filter select bits also determine the ouput data rate
// 1 = Minimum filter, maximum sample rate
int filterSelBits = 20; 


void setup() {

  pinMode (ledPin, OUTPUT);
  digitalWrite (ledPin, 1); // clear the LED

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }


  Serial.println ("AD7124 Multiple sensor type example");

  // For this example I will simply print out the return from each function call
  // so we can see if anything goes wrong 

  // Initializes the AD7124 device
  Serial.print("begin() ");
  Serial.println(adc.begin());

  // Configuring ADC in Full Power Mode (Fastest)
  Serial.print("setAdcControl() ");
  Serial.println(adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true));

 
  // Set the "setup" configurations for different sensor types. There are 7 differnet "setups"
  // in the ADC that can be configured. There are 8 setups that can be configured. Each 
  // setup holds settings for the reference used, the gain setting, filter type, and rate

  Serial.println("setConfig() "); 
  Serial.print("Config 0: "); 
  Serial.println(adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true));    // Load Cell:           External reference tied to excitation, Gain = 128, Bipolar = True
  Serial.print("Config 1: ");
  Serial.println(adc.setup[1].setConfig(AD7124_Ref_Internal, AD7124_Gain_32, true));    // Thermocouple:        Internal reference, Gain = 128, Bipolar = True  
  Serial.print("Config 2: ");
  Serial.println(adc.setup[2].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_1, false));     // Ratiometric Voltage: External reference tied to excitation, Gain = 1, Bipolar = False
  Serial.print("Config 3: ");
  Serial.println(adc.setup[3].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true));     // IC Temp sensor:      Internal reference, Gain = 1, Bipolar = True

  
  Serial.println();

  // Filter settings for each setup. In this case, they are all the same so
  // it could have just been done with a loop
  Serial.println("setConfigFilter() ");
  Serial.print("Filter 0: ");
  Serial.println(adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelBits));
  Serial.print("Filter 1: ");
  Serial.println(adc.setup[1].setFilter(AD7124_Filter_SINC3, filterSelBits));
  Serial.print("Filter 2: ");
  Serial.println(adc.setup[2].setFilter(AD7124_Filter_SINC3, filterSelBits));
  Serial.print("Filter 3: ");
  Serial.println(adc.setup[3].setFilter(AD7124_Filter_SINC3, filterSelBits));
  

  Serial.println();

  // Set channels, i.e. what setup they use and what pins they are measuring on
  Serial.println("setChannel() ");
  Serial.print("Ch 0: ");
  Serial.println(adc.setChannel(0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true)); //Channel 0 - Load cell
  Serial.print("Ch 1: ");
  Serial.println(adc.setChannel(1, 1, AD7124_Input_AIN2, AD7124_Input_AIN3, true)); //Channel 1 - Type K Thermocouple 1
  Serial.print("Ch 2: ");
  Serial.println(adc.setChannel(2, 1, AD7124_Input_AIN4, AD7124_Input_AIN5, true)); //Channel 2 - Type K Thermocouple  
  Serial.print("Ch 3: ");  
  Serial.println(adc.setChannel(3, 2, AD7124_Input_AIN6, AD7124_Input_AVSS, true)); //Channel 3 - Single ended potentiometer 1
  Serial.print("Ch 4: ");  
  Serial.println(adc.setChannel(4, 2, AD7124_Input_AIN7, AD7124_Input_AVSS, true)); //Channel 4 - Single ended potentiometer 2
  Serial.print("Ch 5 [chip temp]:");
  Serial.println(adc.setChannel(5, 3, AD7124_Input_TEMP, AD7124_Input_AVSS, true)); //Channel 5 - ADC IC temperature

  Serial.println();

  // For thermocouples we have to set the voltage bias on the negative input pin 
  // for the channel, in this case it's AIN3
  Serial.print("Setting VBias on channel 1 for thermocouple");
  Serial.println(adc.setVBias(AD7124_VBias_AIN3,true)); 
  Serial.print("Setting VBias on channel 2 for thermocouple");
  Serial.println(adc.setVBias(AD7124_VBias_AIN5,true));  


  // Turn on excitation voltage. If you were only taking readings periodically
  // you could turn this off between readings to save power
  Serial.print("setPWRSW() ");
  Serial.println(adc.setPWRSW(1));
}

// -----------------------------------------------------------------------------

void loop() {

  double readings[CH_COUNT];
  long dt;

  //Take readings, and measure how long it takes just for fun
  dt = micros();

  double junctionTemp = adc.readIcTemp(5); 
  
  readings[0] = adc.readFB(0,2.5, 5.00);  
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