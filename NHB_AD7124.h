/*
  This file is part of the NHB_AD7124 library.

  MIT License

  Copyright (C) 2021  Jaimy Juliano

  Permission is hereby granted, free of charge, to any person obtaining a copy
  of this software and associated documentation files (the "Software"), to deal
  in the Software without restriction, including without limitation the rights
  to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
  copies of the Software, and to permit persons to whom the Software is
  furnished to do so, subject to the following conditions:

  The above copyright notice and this permission notice shall be included in all
  copies or substantial portions of the Software.

  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
  IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
  FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
  AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
  LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
  OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
  SOFTWARE.
*/

#ifndef NHB_AD7124
#define NHB_AD7124

#include <SPI.h>
#include "AD_Defs.h"
#include "Thermocouple.h"


#define AD7124_DEFAULT_TIMEOUT_MS  200 //milliseconds


enum AD7124_OperatingModes {
    AD7124_OpMode_Continuous = 0,           // Continuous conversion mode (default). In continuous conversion mode, the ADC continuously performs conversions and places the result in the data register. 
    AD7124_OpMode_SingleConv,               // Single conversion mode. When single conversion mode is selected, the ADC powers up and performs a single conversion on the selected channel.
    AD7124_OpMode_Standby,                  // Standby mode. In standby mode, all sections of the AD7124 can be powered down except the LDOs. 
    AD7124_OpMode_PowerDown,                // Power-down mode. In power-down mode, all the AD7124 circuitry is powered down, including the current sources, power switch, burnout currents, bias voltage generator, and clock circuitry.
    AD7124_OpMode_Idle,                     // Idle mode. In idle mode, the ADC filter and modulator are held in a reset state even though the modulator clocks continue to be provided.
    AD7124_OpMode_InternalOffsetCalibration,// Internal zero-scale (offset) calibration. An internal short is automatically connected to the input. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
    AD7124_OpMode_InternalGainCalibration,  // Internal full-scale (gain) calibration. A full-scale input voltage is automatically connected to the selected analog input for this calibration. */
    AD7124_OpMode_SystemOffsetCalibration,  // System zero-scale (offset) calibration. Connect the system zero-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
    AD7124_OpMode_SystemGainCalibration     // System full-scale (gain) calibration. Connect the system full-scale input to the channel input pins of the selected channel. RDY goes high when the calibration is initiated and returns low when the calibration is complete.
};

enum AD7124_PowerModes {
    AD7124_LowPower = 0, 
    AD7124_MidPower,     
    AD7124_FullPower     
};

enum AD7124_ClkSources {
    AD7124_Clk_Internal = 0,        // internal 614.4 kHz clock. The internal clock is not available at the CLK pin.
    AD7124_Clk_InternalWithOutput,  // internal 614.4 kHz clock. This clock is available at the CLK pin.
    AD7124_Clk_External,            // external 614.4 kHz clock.
    AD7124_Clk_ExternalDiv4         // external clock. The external clock is divided by 4 within the AD7124.
};

enum AD7124_InputSel {
    AD7124_Input_AIN0 = 0, 
    AD7124_Input_AIN1, 
    AD7124_Input_AIN2, 
    AD7124_Input_AIN3, 
    AD7124_Input_AIN4, 
    AD7124_Input_AIN5, 
    AD7124_Input_AIN6, 
    AD7124_Input_AIN7, 
    AD7124_Input_AIN8, 
    AD7124_Input_AIN9, 
    AD7124_Input_AIN10, 
    AD7124_Input_AIN11, 
    AD7124_Input_AIN12, 
    AD7124_Input_AIN13, 
    AD7124_Input_AIN14, 
    AD7124_Input_AIN15, 
    AD7124_Input_TEMP = 16, // Temperature sensor (internal)
    AD7124_Input_AVSS,      // Connect to AVss
    AD7124_Input_REF,       // Connect to Internal reference 
    AD7124_Input_DGND,      // Connect to DGND.
    AD7124_Input_AVDD6P,    // (AVdd − AVss)/6+. Use in conjunction with (AVdd − AVss)/6− to monitor supply AVdd − AVss . 
    AD7124_Input_AVDD6M,    // (AVdd − AVss)/6−. Use in conjunction with (AVdd − AVss)/6+ to monitor supply AVdd − AVss . 
    AD7124_Input_IOVDD6P,   // (IOVdd − DGND)/6+. Use in conjunction with (IOVdd − DGND)/6− to monitor IOVdd − DGND. 
    AD7124_Input_IOVDD6M,   // (IOVdd − DGND)/6−. Use in conjunction with (IOVdd − DGND)/6+ to monitor IOVdd − DGND. 
    AD7124_Input_ALDO6P,    // (ALDO − AVss)/6+. Use in conjunction with (ALDO − AVss)/6− to monitor the analog LDO. 
    AD7124_Input_ALDO6M,    // (ALDO − AVss)/6−. Use in conjunction with (ALDO − AVss)/6+ to monitor the analog LDO. 
    AD7124_Input_DLDO6P,    // (DLDO − DGND)/6+. Use in conjunction with (DLDO − DGND)/6− to monitor the digital LDO. 
    AD7124_Input_DLDO6M,    // (DLDO − DGND)/6−. Use in conjunction with (DLDO − DGND)/6+ to monitor the digital LDO. 
    AD7124_Input_V20mVP,    // V_20MV_P. Use in conjunction with V_20MV_M to apply a 20 mV p-p signal to the ADC. 
    AD7124_Input_V20mVM     // V_20MV_M. Use in conjunction with V_20MV_P to apply a 20 mV p-p signal to the ADC. 
  };

enum AD7124_GainSel {
    AD7124_Gain_1 = 0,  // Gain 1, Input Range When VREF = 2.5 V: ±2.5 V 
    AD7124_Gain_2,      // Gain 2, Input Range When VREF = 2.5 V: ±1.25 V 
    AD7124_Gain_4,      // Gain 4, Input Range When VREF = 2.5 V: ± 625 mV 
    AD7124_Gain_8,      // Gain 8, Input Range When VREF = 2.5 V: ±312.5 mV 
    AD7124_Gain_16,     // Gain 16, Input Range When VREF = 2.5 V: ±156.25 mV 
    AD7124_Gain_32,     // Gain 32, Input Range When VREF = 2.5 V: ±78.125 mV 
    AD7124_Gain_64,     // Gain 64, Input Range When VREF = 2.5 V: ±39.06 mV 
    AD7124_Gain_128     // Gain 128, Input Range When VREF = 2.5 V: ±19.53 mV 
  };

//NOTE: These are different for the AD7124-8
enum AD7124_VBiasPins{    
    AD7124_VBias_AIN0 = 0x00,
    AD7124_VBias_AIN1 = 0x01,
    AD7124_VBias_AIN2 = 0x04,
    AD7124_VBias_AIN3 = 0x05,
    AD7124_VBias_AIN4 = 0x0A,
    AD7124_VBias_AIN5 = 0x0B,
    AD7124_VBias_AIN6 = 0x0E,
    AD7124_VBias_AIN7 = 0x0F,
};

enum AD7124_RefSources {
	AD7124_Ref_ExtRef1  = 0x00,
	AD7124_Ref_ExtRef2  = 0x01,
	AD7124_Ref_Internal = 0x02,
	AD7124_Ref_Avdd	    = 0x03
};

enum AD7124_Filters {
	AD7124_Filter_SINC4 = 0x00, // SINC4 Filter - Default after reset. This filter gives excellent noise performance over the complete range of output data rates. It also gives the best 50 Hz/60 Hz rejection, but it has a long settling time.
	AD7124_Filter_SINC3 = 0x02, // SINC3 Filter - This filter has good noise performance, moderate settling time, and moderate 50 Hz and 60 Hz (±1 Hz) rejection.
	AD7124_Filter_FAST4 = 0x04, // Fast settling + Sinc4
	AD7124_Filter_FAST3 = 0x05, // Fast settling + Sinc3
	AD7124_Filter_POST  = 0x07  // Post filter enable - The post filters provide rejection of 50 Hz and 60 Hz simultaneously and allow the user to trade off settling time and rejection. These filters can operate up to 27.27 SPS or can reject up to 90 dB of 50 Hz ± 1 Hz and 60 Hz ± 1 Hz interference
};

enum AD7124_PostFilters {
    AD7124_PostFilter_NoPost = 0, // No Post Filter (Default value)
    AD7124_PostFilter_dB47   = 2, // Rejection at 50 Hz and 60 Hz ± 1 Hz: 47 dB, Output Data Rate (SPS): 27.27 Hz 
    AD7124_PostFilter_dB62   = 3, // Rejection at 50 Hz and 60 Hz ± 1 Hz: 62 dB, Output Data Rate (SPS): 25 Hz 
    AD7124_PostFilter_dB86   = 5, // Rejection at 50 Hz and 60 Hz ± 1 Hz: 86 dB, Output Data Rate (SPS): 20 Hz 
    AD7124_PostFilter_dB92   = 6  // Rejection at 50 Hz and 60 Hz ± 1 Hz: 92 dB, Output Data Rate (SPS): 16.7 Hz 
};

enum AD7124_BurnoutCurrents {
    AD7124_Burnout_Off = 0, // burnout current source off (default).
    AD7124_Burnout_500nA,   // burnout current source on, 0.5 μA.
    AD7124_Burnout_2uA,     // burnout current source on, 2 μA.
    AD7124_Burnout_4uA      // burnout current source on, 4 μA.
};

// Excitation currents - Not used yet.
// TODO: Look up actual value in datasheet (IO_CONTROL_1 Register)
enum AD7124_ExCurrents{
    AD7124_ExCurrent_50uA,
    AD7124_ExCurrent_100uA,
    AD7124_ExCurrent_250uA,
    AD7124_ExCurrent_500uA,
    AD7124_ExCurrent_750uA,
    AD7124_ExCurrent_1mA
};

// Device register info
struct Ad7124_Register {
	int32_t addr;
	int32_t value;
	int32_t size;
	int32_t rw;
};

// AD7124 registers ID list
enum AD7124_regIDs {
	Reg_Status = 0x00,
	Reg_Control,
	Reg_Data,
	Reg_IOCon1,
	Reg_IOCon2,
	Reg_ID,
	Reg_Error,
	Reg_Error_En,
	Reg_Mclk_Count,
	Reg_Channel_0,
	Reg_Channel_1,
	Reg_Channel_2,
	Reg_Channel_3,
	Reg_Channel_4,
	Reg_Channel_5,
	Reg_Channel_6,
	Reg_Channel_7,
	Reg_Channel_8,
	Reg_Channel_9,
	Reg_Channel_10,
	Reg_Channel_11,
	Reg_Channel_12,
	Reg_Channel_13,
	Reg_Channel_14,
	Reg_Channel_15,
	Reg_Config_0,
	Reg_Config_1,
	Reg_Config_2,
	Reg_Config_3,
	Reg_Config_4,
	Reg_Config_5,
	Reg_Config_6,
	Reg_Config_7,
	Reg_Filter_0,
	Reg_Filter_1,
	Reg_Filter_2,
	Reg_Filter_3,
	Reg_Filter_4,
	Reg_Filter_5,
	Reg_Filter_6,
	Reg_Filter_7,
	Reg_Offset_0,
	Reg_Offset_1,
	Reg_Offset_2,
	Reg_Offset_3,
	Reg_Offset_4,
	Reg_Offset_5,
	Reg_Offset_6,
	Reg_Offset_7,
	Reg_Gain_0,
	Reg_Gain_1,
	Reg_Gain_2,
	Reg_Gain_3,
	Reg_Gain_4,
	Reg_Gain_5,
	Reg_Gain_6,
	Reg_Gain_7,
	Reg_REG_NO
};

// Struct to store the setup values for easy access. This may go away, it's memory wastefull
// and the info could be extracted from the register info we already have stored
// (only used for readVolts now)
struct Ad7124_SetupVals{
    //Config
    AD7124_RefSources ref = AD7124_Ref_ExtRef1;
    AD7124_GainSel gain = AD7124_Gain_1;
    bool bipolar = true;
    AD7124_BurnoutCurrents burnout = AD7124_Burnout_Off;

    //Filter
    AD7124_Filters filter = AD7124_Filter_SINC4;
    uint16_t fs = 0;
    AD7124_PostFilters postfilter;
    bool rej60 = false;
    bool single = false;

    //Cal coefficients
    uint32_t offsetCoeff;
    uint32_t gainCoeff;

    double refV = 2.500; //Can be set to accomodate a different external ref voltage
};

class Ad7124;


//Class to manage the AD7124 "setups"
class Ad7124Setup{    
    
    public:
        
        void init(Ad7124* driver, uint8_t index) {_driver = driver; setupNum = index;}; 

              
        //Sets configuration reg values
        int setConfig (AD7124_RefSources ref, AD7124_GainSel gain, bool bipolar, AD7124_BurnoutCurrents burnout = AD7124_Burnout_Off, double exRefV = 2.50); 

        //Sets the filter type and output word rate for a setup
        int setFilter (AD7124_Filters filter, uint16_t fs, AD7124_PostFilters postfilter = AD7124_PostFilter_NoPost, bool rej60 = false, bool single = false); 

        //Set offset for a setup
        int setOffsetCal (uint32_t value); 

        //Set gain for a setup
        int setGainCal (uint32_t value); 


        //Return the reference voltage
        double refV();

        //Return if we are using bipolar mode
        bool bipolar();

        //Return the current gain setting
        uint8_t gain();
        
    private:
        uint8_t setupNum;
        
        Ad7124* _driver; //pointer to outer class instance  
        Ad7124_SetupVals setupValues;      
};


class Ad7124 {    
    
    public:      

        Ad7124(uint8_t csPin, uint32_t spiFrequency);
                
        int begin();
        int reset();

        
        //Enable or disable the onboard PWR_SW FET
        int setPWRSW(bool enabled);

        //Read a single channel in single conversion mode and
        //return the value in raw ADC counts
        int32_t readRaw(uint8_t ch);

        //Read each enabled channel in single conversion mode and
        //return the values in raw ADC counts
        int readRaw(int32_t *buf, uint8_t chCount); 

        //Read a single channel in single conversion mode 
        //return the value in voltage
        double readVolts(uint8_t ch); 

        //Read multiple channels in single conversion mode
        //return the readings in voltage
        int readVolts(double *buf, uint8_t chCount); 

        //Read thermocouple. Currently only Type K is supported.
        //The channel must first be setup properly for reading thermocouples.
        double readTC(uint8_t ch, double refTemp, TcTypes type = Type_K);

        //Read a 4 wire full bridge sensor. Return value can be scaled with
        //optional scaleFactor arg. Returns mV/V if scale factor is one (default)
        double readFB(uint8_t ch, double vEx, double scaleFactor = 1.000);

        //Read the on chip temp sensor. EXPERIMENTAL
        double readIcTemp(uint8_t ch);

        //Enable bias voltage on given channel
        int setVBias(AD7124_VBiasPins vBiasPin, bool enabled); 

        //Set excitation current
        //int setExCurrent(uint8_t ch, AD7124_ExCurrents); //NOT IMPLEMENTED YET

        void setTimeout(uint32_t ms) {timeout = ms;}
        
        
        //Sets the ADC Control register
        int setAdcControl (AD7124_OperatingModes mode, AD7124_PowerModes power_mode, bool ref_en = true, AD7124_ClkSources clk_sel = AD7124_Clk_Internal); 

        //Control the mode of operation for ADC 
        int setMode (AD7124_OperatingModes mode); 



        //Configure a channel
        int setChannel (uint8_t ch, uint8_t setup, AD7124_InputSel aiPos, AD7124_InputSel aiNeg, bool enable = false); 

        //Enable/Disable channel  
        int enableChannel (uint8_t ch, bool enable = true); 

        //Returns the setup number used by the channel
        int channelSetup (uint8_t ch); 

        

        //Start conversion in single mode
        int startSingleConversion (uint8_t ch); 

        //Waits until a new conversion result is available.
        int waitEndOfConversion (uint32_t timeout_ms); 

        //Returns the last sampling channel
        int currentChannel(); 
        
        //Return the the status register contents
        int status();

        //Returns the most recent sample in raw ADC counts with 
        //status bits appended (data + status mode)
        int32_t getData();

        //Testing version to use when status bits are appended to data
        //int32_t getData2();
        
        //Converts raw ADC counts to voltage. The conversion
        //is dependent on the gain, ref voltage, and bipolar mode
        double toVolts (long value, int gain, double vref, bool bipolar);

        //Converts a raw reading from the on chip temp sensor to degrees C
        double tempSensorRawToDegC(long value); 
        
        Ad7124Setup setup[8];


    private:
                
        int noCheckReadRegister(Ad7124_Register *reg);
        int noCheckWriteRegister(Ad7124_Register reg);

        int32_t readRegister(AD7124_regIDs id); 
        int writeRegister(AD7124_regIDs id);

        int waitForConvReady(uint32_t timeout);
        int waitForSpiReady(uint32_t timeout);
        int waitForPowerOn(uint32_t timeout);

        void updateCRCSetting(void);
        uint8_t computeCRC8(uint8_t *buffer, uint8_t size);

        void updateDevSpiSettings(void);
        void spiWriteAndRead(uint8_t *data, uint8_t numBytes); 

       
        SPISettings spiSettings;
        Thermocouple thermocouple;
                
        bool crcEnabled = false;
        bool isReady = true; //Not really used now, may go away [8-26-21]
        uint8_t cs;
        uint32_t timeout = AD7124_DEFAULT_TIMEOUT_MS;

        //Init regs struct with power on default values
        Ad7124_Register regs[Reg_REG_NO] = {
            {0x00, 0x00,   1, 2}, /* Status */
            {0x01, 0x0000, 2, 1}, /* ADC_Control */
            {0x02, 0x0000, 3, 2}, /* Data */
            {0x03, 0x0000, 3, 1}, /* IOCon1 */
            {0x04, 0x0000, 2, 1}, /* IOCon2 */
            {0x05, 0x02,   1, 2}, /* ID */
            {0x06, 0x0000, 3, 2}, /* Error */
            {0x07, 0x0044, 3, 1}, /* Error_En */
            {0x08, 0x00,   1, 2}, /* Mclk_Count */
            {0x09, 0x8001, 2, 1}, /* Channel_0 */
            {0x0A, 0x0001, 2, 1}, /* Channel_1 */
            {0x0B, 0x0001, 2, 1}, /* Channel_2 */
            {0x0C, 0x0001, 2, 1}, /* Channel_3 */
            {0x0D, 0x0001, 2, 1}, /* Channel_4 */
            {0x0E, 0x0001, 2, 1}, /* Channel_5 */
            {0x0F, 0x0001, 2, 1}, /* Channel_6 */
            {0x10, 0x0001, 2, 1}, /* Channel_7 */
            {0x11, 0x0001, 2, 1}, /* Channel_8 */
            {0x12, 0x0001, 2, 1}, /* Channel_9 */
            {0x13, 0x0001, 2, 1}, /* Channel_10 */
            {0x14, 0x0001, 2, 1}, /* Channel_11 */
            {0x15, 0x0001, 2, 1}, /* Channel_12 */
            {0x16, 0x0001, 2, 1}, /* Channel_13 */
            {0x17, 0x0001, 2, 1}, /* Channel_14 */
            {0x18, 0x0001, 2, 1}, /* Channel_15 */
            {0x19, 0x0860, 2, 1}, /* Config_0 */
            {0x1A, 0x0860, 2, 1}, /* Config_1 */
            {0x1B, 0x0860, 2, 1}, /* Config_2 */
            {0x1C, 0x0860, 2, 1}, /* Config_3 */
            {0x1D, 0x0860, 2, 1}, /* Config_4 */
            {0x1E, 0x0860, 2, 1}, /* Config_5 */
            {0x1F, 0x0860, 2, 1}, /* Config_6 */
            {0x20, 0x0860, 2, 1}, /* Config_7 */
            {0x21, 0x060180, 3, 1}, /* Filter_0 */
            {0x22, 0x060180, 3, 1}, /* Filter_1 */
            {0x23, 0x060180, 3, 1}, /* Filter_2 */
            {0x24, 0x060180, 3, 1}, /* Filter_3 */
            {0x25, 0x060180, 3, 1}, /* Filter_4 */
            {0x26, 0x060180, 3, 1}, /* Filter_5 */
            {0x27, 0x060180, 3, 1}, /* Filter_6 */
            {0x28, 0x060180, 3, 1}, /* Filter_7 */
            {0x29, 0x800000, 3, 1}, /* Offset_0 */
            {0x2A, 0x800000, 3, 1}, /* Offset_1 */
            {0x2B, 0x800000, 3, 1}, /* Offset_2 */
            {0x2C, 0x800000, 3, 1}, /* Offset_3 */
            {0x2D, 0x800000, 3, 1}, /* Offset_4 */
            {0x2E, 0x800000, 3, 1}, /* Offset_5 */
            {0x2F, 0x800000, 3, 1}, /* Offset_6 */
            {0x30, 0x800000, 3, 1}, /* Offset_7 */
            {0x31, 0x500000, 3, 1}, /* Gain_0 */
            {0x32, 0x500000, 3, 1}, /* Gain_1 */
            {0x33, 0x500000, 3, 1}, /* Gain_2 */
            {0x34, 0x500000, 3, 1}, /* Gain_3 */
            {0x35, 0x500000, 3, 1}, /* Gain_4 */
            {0x36, 0x500000, 3, 1}, /* Gain_5 */
            {0x37, 0x500000, 3, 1}, /* Gain_6 */
            {0x38, 0x500000, 3, 1}, /* Gain_7 */
            };

        friend class Ad7124Setup;
            
};
    
#endif