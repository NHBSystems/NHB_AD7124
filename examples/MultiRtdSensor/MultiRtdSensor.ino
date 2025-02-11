/*
  AD7124 Multiple RTD Sensor Example
  
  Reads 2 RTD Sensors as per https://www.analog.com/en/_/media/analog/en/reference-circuits/cn0383/cn0383_16_1024.gif

  Setup in 3-wire mode and requires 2 current excitation sources.
  Probe 0
  -AIN0 Excitation source 0
  -AIN2 Probe +
  -AIN3 Probe -
  -AIN1 Excitation source 1
  -3rd wire tied to RREF

  Probe 1
  -AIN6 Excitation source 0
  -AIN4 Probe +
  -AIN5 Probe -
  -AIN7 Excitation source 1
  -3rd wire tied to RREF

  For more on AD7124, see
  http://www.analog.com/media/en/technical-documentation/data-sheets/AD7124-4.pdf

  This file is part of the NHB_AD7124 library.

  MIT License - A copy of the full text should be included with the library

  Copyright (C) 2025  Aaron Neal

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
const long FullScale = 1L << 24;

void setup() {

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only

  Serial.println ("AD7124 2 channel RTD example");

  // Initializes the AD7124 device
  adc.begin();
  
  // Configuring ADC in Full Power Mode (Fastest)
  adc.setAdcControl(AD7124_OpMode_Continuous, AD7124_FullPower, true); 

  //setup probe 0
  adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_4, true, AD7124_Burnout_Off); 
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelBits, AD7124_PostFilter_NoPost, true, false);
  adc.setChannel(0, 0, AD7124_Input_AIN2, AD7124_Input_AIN3, false);

  //setup probe 1
  adc.setup[1].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_4, true, AD7124_Burnout_Off); 
  adc.setup[1].setFilter(AD7124_Filter_SINC3, filterSelBits, AD7124_PostFilter_NoPost, true, false);
  adc.setChannel(1, 1, AD7124_Input_AIN4, AD7124_Input_AIN5, false);
}

//we can use burnout to detect disconnected probes
bool burnoutDetect(uint8_t probe){
  adc.setup[probe].setConfig(AD7124_Ref_Internal, AD7124_Gain_1, true, AD7124_Burnout_4uA); 
  delay(5); //wait for burnout current to stabilize
  double rawAdc = adc.readRaw(probe);
  adc.setup[probe].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_4, true, AD7124_Burnout_Off); //return back to normal settings
  if(rawAdc >= (FullScale-10)){
    return true;
  } else {
    return false;
  }
}

void readProbe(uint8_t probe){
  Serial.printf("Probe %d:", probe);

  //enable setup
  adc.enableChannel(probe, true);
  bool openCircuit = burnoutDetect(probe);

  if(openCircuit){
    Serial.println("Open Circuit Error");
  } else {
    //set correct excitation for each probe
    if(probe == 0){
      adc.setExCurrent(AD7124_ExCurrentOutputChannel_AIN0, AD7124_ExCurrentSource_0, AD7124_ExCurrent_250uA); //probe 1
      adc.setExCurrent(AD7124_ExCurrentOutputChannel_AIN1, AD7124_ExCurrentSource_1, AD7124_ExCurrent_250uA); //probe 1
    } else {
      adc.setExCurrent(AD7124_ExCurrentOutputChannel_AIN6, AD7124_ExCurrentSource_0, AD7124_ExCurrent_250uA); //probe 1
      adc.setExCurrent(AD7124_ExCurrentOutputChannel_AIN7, AD7124_ExCurrentSource_1, AD7124_ExCurrent_250uA); //probe 1
    }

    delay(20); //wait for stabilisation

    double rawAdc = adc.readRaw(probe); //read the ADC

    //turn currents off, doesn't matter the channel, there are only 2 sources
    adc.setExCurrent(AD7124_ExCurrentOutputChannel_AIN0, AD7124_ExCurrentSource_0, AD7124_ExCurrent_Off);
    adc.setExCurrent(AD7124_ExCurrentOutputChannel_AIN1, AD7124_ExCurrentSource_1, AD7124_ExCurrent_Off);

    //disable setup
    adc.enableChannel(probe, false);

    double resistance = adc.rtd.toResistance(rawAdc1, 4, 5110) * 2; //multiply by 2 because we are using a 2 excitation sources (current is double)
    double temp = adc.rtd.toTemperature(resistance); //get temperature from resistance,
    
    Serial.print(resistance, 2);
    Serial.print(',');
    Serial.print(temp, 2);
    Serial.println();
  }
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
