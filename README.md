# NHB_AD7124  

Arduino Library for the Analog Devices AD7124 24bit ADC

The AD7124-4 is a 4 channel, 24 bit, differential ADC (it can also be configured as 7 single ended channels).
The library was originally written for use with the AD7124 Analog Sensor FeatherWing, but there is no reason it couldn't be used with a raw chip in your own design.

This library has only been tested with SAMD21 so far, but it really should work with any architectures that have a working, Arduino style SPI implementation. 

Basic API  
--------


### Constructor   

```c
Ad7124(uint8_t csPin, uint32_t spiFrequency);
```
| Arg           |  Description    |
| -----------   | --------------- |
|*csPin*        | Chip Select pin |
|*spiFrequency* | SPI bus frequency to use|


You will also need to cal the standard Arduino style begin method in your setup() function to initialize the Ad7124 chip. It takes no arguments.
```c
begin();
```


-------------------------

### ADC Configuration  

To set the basic global parameters of the ADC we use the `setAdcControl(..)` method

```c
int setAdcControl (AD7124_OperatingModes   mode, 
                   AD7124_PowerModes power_mode,
                   bool ref_en = true, 
                   AD7124_ClkSources clk_sel = AD7124_Clk_Internal); 
```

<!--- Commented out for now because I'm not sure about this. It makes the raw text pretty ugly
|Argument|Description|Valid Options|
| ---------------| -----------------------------|----------------------------------|
| `mode`         | Sets the operating mode      | `AD7124_OpMode_Continuous`<br>`AD7124_OpMode_SingleConv` <br>`AD7124_OpMode_Standby` <br>`AD7124_OpMode_PowerDown` <br>`AD7124_OpMode_Idle` <br>`AD7124_OpMode_InternalOffsetCalibration` <br>`AD7124_OpMode_InternalGainCalibration` <br>`AD7124_OpMode_SystemOffsetCalibration` <br>`AD7124_OpMode_SystemGainCalibration`|
|`power_mode`    | Sets the power mode          | `AD7124_LowPower` <br> `AD7124_MidPower` <br> `AD7124_FullPower` |
| `ref_en`       | Sets if the internal reference voltage is enabled<br> *Optional argument, Defaults to true*| `true` <br> `false`. 
| `clk_sel`      | Selects clock source the Ad7124 will use<br> *Optional argument, defaults to AD7124_Clk_Internal*| `AD7124_Clk_Internal`<br> `AD7124_Clk_InternalWithOutput`<br> `AD7124_Clk_External`<br> `AD7124_Clk_ExternalDiv4`

--------------------------
--->


|Argument| Description|
| ---------------| -------------------------------------------- |
| `mode`         | Sets the operating mode of the device.       |
| `power_mode`   | Sets the power mode                          |
| `ref_en`       | Sets if the internal reference voltage is enabled<br> *Optional argument, Defaults to true*|
| `clk_sel`      | Selects clock source the Ad7124 will use<br> *Optional argument, defaults to AD7124_Clk_Internal*| 


### AD7124 "Setups"  


The AD7124-4 ICs have a feature defined in the datasheet as "Setups". The Setups allow for preconfiguring for different sensor types, independent of which channel they are asigned to. The setups are modeled in the library as an array of setup objects contained within the Ad7124 class. There are 8 individual setups that can be used to hold different configurations.  

The two main methods for configuring a setup are `setConfig(..)`, and `setFilter(..)`  

```cpp
int Ad7124Setup::setConfig(AD7124_RefSources ref, AD7124_GainSel gain,
                           bool bipolar, AD7124_BurnoutCurrents burnout,
                           double exRefV = 2.50);
```


|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ref`          | Selects what reference voltage will be used               |
| `gain`         | Selects what gain will be used |
| `bipolar`      | Sets for bipolar or unipolar input |
| `burnout`      | Selects burnout current option <br> *Optional argument, defaults to AD7124_Burnout_Off* |
| `exRefV`       | Set the reference voltage, if an external reference is used. This value is used internally for calculating voltage from raw ADC counts.<br> *Optional argument, defaults to 2.50* |
----------------------

```cpp
int Ad7124Setup::setFilter(AD7124_Filters filter, uint16_t fs,
                           AD7124_PostFilters postfilter, bool rej60,
                           bool single); 
```

|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `filter`       | Select which filter type to use                           |
| `fs`           | Filter output rate select bits. can be a value from 1 to 2047. Setting to 1 will give fastest output for the selcted filter. See datasheet for details |
| `postfilter`   | Selects a post filter option <br> *Optional argument, defaults to AD7124_PostFilter_NoPost* |
| `rej60`        | Enables a first order notch at 60 Hz when the first notch of the sinc filter is at 50 Hz, allowing simultaneous 50 and 60 Hz rejection. <br>*Optional argument, defaults to false* |
| `single`       | Enables "single cycle conversion". Has no effect when using multiple channels or when in single conversion mode. <br> *Optional argument, defaults to false* |

--------------------------

### Channel Configuration

We need need to define what inputs are used by each channel and what setup will be used. This is done with the `setChannel(..)` method.

```cpp
int setChannel (uint8_t ch, uint8_t setup, AD7124_InputSel aiPos, 
                AD7124_InputSel aiNeg, bool enable); 

```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel we are configuring                            |
| `setup`        | Which setup will this channel use?                        |
| `aiPos`        | The pin that will be connected to the positive analog input for this channel |
| `aiNeg`        | The pin that will be connected to the negative analog input for this channel |
| `enable`       | Set if the channel is enabled or not <br> *Optional argument, defaults to false* |

<br/><br/>

### Getting Readings

There are a few different methods to get readings from the ADC. The most obvious one is readVolts(..). The basic version simply returns the voltage on the given channel. There is also an overloaded version that will read a sequential group of channels into a buffer.
  
`readVolts(ch)` Returns a voltage reading for the given channel
```cpp 
double readVolts(uint8_t ch);
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read.                                      |  

  
`readVolts(buf,chCount)` Can be used to read a nuber of channels at once, though they must start at 0 and be sequential. (e.g. 0 trough 3, or 0 trough 5).  

```cpp
int readVolts(double *buf, uint8_t chCount);
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `buf`          | Buffer. An array large enough to hold the number of channels given by the chCount argument |
| `chCount`      | The number of channels to read.                               |  

<br>

The readRaw(..) methods are provided for lower level access to the raw ADC counts. Like readVolts() there is an overloaded version that will read multiple channels into a buffer.
  
The `readRaw(chan)` Returns the raw ADC counts for the given channel  

```cpp
int32_t readRaw(uint8_t ch);
```  
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read.                                      |  


`readRaw(buf,chCount)` Can be used to read a nuber of channels at once, though they must start at 0 and be sequential. (e.g. 0 trough 3, or 0 trough 5). 

```cpp
int readRaw(int32_t *buf, uint8_t chCount);
```  
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `buf`          | Buffer. An array large enough to hold the number of channels given by the chCount argument |
| `chCount`      | The number of channels to read.                               |  

<br>

There are also a few sensor specific read functions provided for convenience.  
  
The `readTC(ch,refTemp, type)` method is intended to simplify reading thermocouples. It currently only supports Type K thermocouples, but I intend to add more types in the future. 

```cpp
double readTC(uint8_t ch, double refTemp, TcTypes type)
```  
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read                                       |
| `refTemp`      | The reference (cold junction) temperature.                |
| `type`         | The type of thermocouple you are reading. *(Currently only Type K is supported)* <br> *Optional argument, defaults to Type_K* |

<br>

The `readFB(ch, vEx, scaleFactor)` method is for reading full bridge type sensors (load cells, pressure gauges, extensometers, ...). It could also be used for potentiometers as long as you have setup the channel properly. Returns mV/V if scale factor is one (default)

```cpp
double readFB(uint8_t ch, double vEx, double scaleFactor);
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel to read                                       |
| `vEx`          | The excitation voltage being used. (2.50 for built in regulator on NHB boards) |
| `scaleFactor`  | A linear scaling factor to apply to the reading. A factor of 1.00 will return the reading in mV/V. <br> *Optional argument, defaults to 1.00* |

<br>

The `readIcTemp(ch)` reads the temperature sensor embeded in the AD7124. The channel must be properly configured to read the sensor first.

```cpp
double readIcTemp(uint8_t ch);
```
|   Argument     |    Description                                            |
| ---------------| --------------------------------------------------------- |
| `ch`           | The channel that is configured to read the internal temperature sensor |

------------------------  

<br/><br/>  

Example
-----------
```cpp
#include "NHB_AD7124.h"

// Read a full bridge sensor and print mV/V readings out the serial port

// Change for your hardware
const uint8_t ssPin = 10;

// Filter output rate selection bits, needs to be between 1 and 2047
uint16_t filterSelectVal = 320;

Ad7124 adc(ssPin, 4000000);



void setup() {
  
  //Initialize serial and wait for port to open:
  Serial.begin (115200);
  while (!Serial) {;} // wait for serial port to connect. Needed for native USB port only

  
  adc.begin();
  
  adc.setAdcControl(AD7124_OpMode_SingleConv, AD7124_FullPower, true);

  // Setting configuration for Setup 0:
  // - Use the external reference tied to the excitation voltage (2.5V reg)
  // - Set a gain of 128 and bipolar to true for a measurement range of +/- 19.53 mv    
  adc.setup[0].setConfig(AD7124_Ref_ExtRef1, AD7124_Gain_128, true);
  adc.setup[0].setFilter(AD7124_Filter_SINC3, filterSelectVal);

  // Setting channel 0 using pins AIN1(+)/AIN0(-) and enable
  adc.setChannel(0, 0, AD7124_Input_AIN0, AD7124_Input_AIN1, true);

  // Turn on excitation voltage regulator.
  adc.setPWRSW(1);
}

void loop() {

  double reading;
  
  // Using 2.5V excitation with 1.00 scaling to simply return mV/V
  reading = adc.readFB(0, 2.50, 1.00); 

  Serial.println(reading, DEC);
    
}

```