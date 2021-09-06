/*
  AD7124 read load cell on channel 0 as fast as we can. Note this will most likely
  produce very noisy results.

  For more on AD7124, see
  http://www.analog.com/media/en/technical-documentation/data-sheets/AD7124-4.pdf

*/

#include <NHB_AD7124.h>


const uint8_t ledPin = 13;
const uint8_t ssPin = 10;

Ad7124 adc(ssPin, 4000000);

// TODO: Figure out actual update rates
uint16_t filterSelBits = 1;


void setup() {

  pinMode (ledPin, OUTPUT);
  digitalWrite (ledPin, 1); // clear the LED

  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only

  // prints title with ending line break
  Serial.println ("AD7124 1 channel test");

  // Initializes the AD7124 device
  Serial.print("begin() ");
  Serial.println(adc.begin());
  

  // Setting the configuration 0:
  // - use the external reference tied to the excitation voltage (2.5V reg)
  // - gain of 128 for a bipolar measurement of +/- 19.53 mv
  Serial.print("setConfig() ");  
  //Serial.println(adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true, AD7124_Burnout_Off, 2.50)); //With optional args
  Serial.println(adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true));

  Serial.print("setFilter() ");
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelBits);

  // Setting channel 0 using pins AIN1(+)/AIN0(-)
  Serial.print("setChannel() ");
  Serial.println(adc.setChannel (0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true));

  // Configuring ADC in Full Power Mode (Fastest)
  Serial.print("setAdcControl() ");
  Serial.println(adc.setAdcControl (AD7124_OpMode_SingleConv, AD7124_FullPower, true));

  // Turn on excitation voltage
  Serial.print("setPWRSW() ");
  Serial.println(adc.setPWRSW(1));
}


void loop() {
  double voltage;
  long dt;
  

  // Measuring Voltage on Channel 0 in Single Conversion Mode
  digitalWrite (ledPin, 0);

  dt = micros();  
  
  voltage = adc.readVolts(0); 
    
  dt = micros() - dt;

  digitalWrite (ledPin, 1);


  // If the measurement is successful, the value is converted into voltage
  //voltage = adc.toVolts (value, 128, 2.5, true);    
  // Print result
  Serial.print(voltsToEngUnits(voltage,2.5,5), DEC);
  Serial.print('\t');
  Serial.print("dt=");
  Serial.print(dt);
  Serial.print("uS");
  Serial.println();
  
}
/* ========================================================================== */

//For full bridge sensor, convert to mV/V and apply linear scailing
float voltsToEngUnits(float vIn, float vEx, float scaleFactor){
  
  float mVpV = (vIn * 1000) / vEx;
  return mVpV * scaleFactor;
}