#include "NHB_AD7124.h"


/* Error codes */
#define AD7124_INVALID_VAL -1 /* Invalid argument */
#define AD7124_COMM_ERR    -2 /* Communication error on receive */
#define AD7124_TIMEOUT     -3 /* A timeout has occured */


// Sets configuration reg values 
int Ad7124Setup::setConfig (AD7124_RefSources ref, AD7124_GainSel pga, bool bipolar, AD7124_BurnoutCurrents burnout, double exRefV){
    
    //Store these away for easy access later (this is a bit redundant, should proabably just get values from reg struct when needed)
    setupValues.ref = ref;
    setupValues.pga = pga;
    setupValues.bipolar = bipolar;
    setupValues.burnout;

    if(ref < AD7124_Ref_Internal){
      setupValues.refV = exRefV;
    }

    //Offset to config reg group
    uint8_t reg = setupNum + Reg_Config_0;    

    _driver->regs[reg].value = AD7124_CFG_REG_REF_SEL (ref) |
                                    AD7124_CFG_REG_PGA (pga) |
                                    (bipolar ? AD7124_CFG_REG_BIPOLAR : 0) |
                                    AD7124_CFG_REG_BURNOUT (burnout) |
                                    AD7124_CFG_REG_REF_BUFP | AD7124_CFG_REG_REF_BUFM |
                                    AD7124_CFG_REG_AIN_BUFP | AD7124_CFG_REG_AINN_BUFM;

    return _driver->writeRegister((AD7124_regIDs) reg);
}

// Sets the filter type and output word rate for a setup 
int Ad7124Setup::setFilter (AD7124_Filters filter, uint16_t fs, AD7124_PostFilters postfilter, bool rej60, bool single){

  // Store these away for easy access later 
  // (this is a bit redundant, should proabably just get values from reg struct when needed)
  setupValues.filter = filter;
  setupValues.fs = fs;
  setupValues.postfilter = postfilter;
  setupValues.rej60 = rej60;
  setupValues.single = single;  

  //Offset to filter reg group
  uint8_t reg = setupNum + Reg_Filter_0;  
  
  _driver->regs[reg].value =  AD7124_FILT_REG_FILTER ( (uint32_t) filter) |
                              AD7124_FILT_REG_POST_FILTER ( (uint32_t) postfilter) |
                              AD7124_FILT_REG_FS (fs)    |
                              (rej60 ? AD7124_FILT_REG_REJ60 : 0) |
                              (single ? AD7124_FILT_REG_SINGLE_CYCLE : 0);

  return _driver->writeRegister( (AD7124_regIDs) reg);
   
}

// Set offset calibration for a setup 
int Ad7124Setup::setOffsetCal (uint32_t value){

  // Store this away for easy access later 
  // (this is a bit redundant, should proabably just get values from reg struct when needed)
  setupValues.offsetCoeff = value;

  // Offset to offset reg group
  uint8_t reg = setupNum + Reg_Offset_0;

  _driver->regs[reg].value = value;
  return _driver->writeRegister((AD7124_regIDs) reg);
}

// Set gain calibration for a setup 
int Ad7124Setup::setGainCal (uint32_t value){
  //Store this away for easy access later
  setupValues.gainCoeff = value;

  //Offset to gain reg group
  uint8_t reg = setupNum + Reg_Gain_0;
  
  _driver->regs[reg].value = value;
  return _driver->writeRegister((AD7124_regIDs) reg);
}

// Return the reference voltage 
double Ad7124Setup::refV(){
  return setupValues.refV;
}

// Return if we are using bipolar mode 
bool Ad7124Setup::bipolar(){
  return setupValues.bipolar;
}

// Return the current gain setting 
uint8_t Ad7124Setup::gain(){  
  //We can left shift 1 by the values of the AD7124_GainSel enum elements
  //to yeald the actual gain value (e.g.  1 << AD7124_Gain_128 = 128) 
  return  1 << (uint8_t) setupValues.pga;
}


Ad7124::Ad7124(uint8_t csPin, uint32_t spiFrequency){
    cs = csPin;

    spiSettings = SPISettings(spiFrequency, MSBFIRST, SPI_MODE3);

    //isReady = false; [Commented out 8-26-21]
    crcEnabled = false;

    for(int i = 0; i < 8; i++){      
      setup[i].init(this, i); // initialize setup objects and tell them thier index number
    }
            
}

///////////////////////////////////////////////////////////////////////////////////////////
int Ad7124::begin(){
    int ret;
    
    pinMode(cs, OUTPUT);
    SPI.begin();

    //return setAdcControl (AD7124_OpMode_Standby, AD7124_FullPower, false); // Wait, why is this here? [8-26-21]
    return reset();     
}

/////////////////////////////////////////////////////////////////////////////////
int Ad7124::reset (void) {
  int ret = 0;

  // Write 64 1's to reset the chip
  uint8_t wrBuf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};

  // NOTE: Arduino SPI transfer doesn't return any error codes so no check here
  spiWriteAndRead(wrBuf, 8);
   

  // Wait for the reset to complete 
  return waitForPowerOn (timeout);
}

//////////////////////////////////////////////////////////////////////////////////
int Ad7124::setPWRSW(bool enabled) {
  
  regs[Reg_IOCon1].value &= ~ AD7124_IO_CTRL1_REG_PDSW;
  
  if (enabled == true) {
    regs[Reg_IOCon1].value |= AD7124_IO_CTRL1_REG_PDSW;
  }
 
  return writeRegister(Reg_IOCon1);
}

//////////////////////////////////////////////////////////////////////////////////
int Ad7124::setVBias(AD7124_VBiasPins vBiasPin, bool enabled){
  
  regs[Reg_IOCon2].value &= ~ (1 << (uint8_t) vBiasPin);
  
  if (enabled == true) {
    regs[Reg_IOCon2].value |= (1 << (uint8_t) vBiasPin);
  }
 
  return writeRegister(Reg_IOCon2);
}

// Get a reading in raw counts from a single channel 
int32_t Ad7124::readRaw(uint8_t ch) {
  int ret;
  uint8_t cur_ch = currentChannel(); // <-- Remember, this only works properly when using Data + Status mode
    
  if (ch != cur_ch) {
    // disable previous channel if different
    ret = enableChannel(cur_ch, false);
    if (ret < 0) {
      return ret;
    }

    //Moved here so only called if channel changed
    ret = startSingleConversion(ch); //This calls enable channel internally
    if (ret < 0) {
      return ret;
    }
  }
  // If no channel change, just call setMode
  else{
    ret = setMode(AD7124_OpMode_SingleConv);
    if (ret < 0) {
      return ret;
    }
  }
  
  ret = waitForConvReady(timeout);
  if (ret < 0) {
    return ret;
  }

  return getData();  
}

// Get a reading in raw counts from all enabled channels 
// Multiple channels should be enabled before calling
int Ad7124::readRaw(int32_t *buf, uint8_t chCount) {
  int ret;
  int ch;
  int32_t data;
  
  ret = setMode(AD7124_OpMode_SingleConv);
  
  if (ret < 0) {
    return ret;
  }

  for(int i = 0; i < chCount; i++){
    
    waitForConvReady(timeout);
    if (ret < 0) {
      return ret;
    }
    
    data = getData();
    
    ch = currentChannel(); //This works now, but using as the buffer index wouldnt work if we skipped a channel number. WHAT TO DO?
        
    buf[i] = data; // Keep using the loop index for now. I need to think about this some more. So far it has worked fine this way
    //buf[ch] = data; 

    if(buf[i] < 0){
      return AD7124_COMM_ERR;
    }    
  }

  // Serial.println();

  return 0;
}

// Get a reading in voltage from a single channel 
double Ad7124::readVolts(uint8_t ch){

  uint8_t idx = channelSetup(ch);
    
  return toVolts(readRaw(ch), setup[idx].gain(), setup[idx].refV(), setup[idx].bipolar());
}

// Get readings for multiple sequential channels in volts 
int Ad7124::readVolts(double *buf, uint8_t chCount){
  int ret;  
  int32_t data[8];
  
  ret = readRaw(data, chCount);
  if (ret < 0) {
    return ret;
  }

  for(int i = 0; i < chCount; i++){    
      
    uint8_t idx = channelSetup(i);

    // If the measurement is successful, the value is converted into voltage
    buf[i] = toVolts(data[i], setup[idx].gain(), setup[idx].refV(), setup[idx].bipolar());     
          
  }

  return 0;
}

// Sets the ADC Control register 
int Ad7124::setAdcControl (AD7124_OperatingModes mode, AD7124_PowerModes power_mode, bool ref_en, AD7124_ClkSources clk_sel){
  
  regs[Reg_Control].value = AD7124_ADC_CTRL_REG_MODE (mode) |
                            AD7124_ADC_CTRL_REG_POWER_MODE (power_mode) |
                            AD7124_ADC_CTRL_REG_CLK_SEL (clk_sel) |
                            (ref_en ? AD7124_ADC_CTRL_REG_REF_EN : 0) |
                            AD7124_ADC_CTRL_REG_DATA_STATUS | //Enable data + status mode
                            AD7124_ADC_CTRL_REG_DOUT_RDY_DEL;

  return writeRegister(Reg_Control);
}

// Control the mode of operation for ADC 
int Ad7124::setMode (AD7124_OperatingModes mode){
   
    regs[Reg_Control].value &= ~AD7124_ADC_CTRL_REG_MODE (0x0F); // clear mode
    regs[Reg_Control].value |= AD7124_ADC_CTRL_REG_MODE (mode);
    
    return writeRegister(Reg_Control);
} 

// Configure channel 
int Ad7124::setChannel (uint8_t ch, uint8_t cfg, AD7124_InputSel ainp, AD7124_InputSel ainm, bool enable){
  if ( (ch < 16) && (cfg < 8)) {
    
    ch += Reg_Channel_0;
    
    regs[ch].value =  AD7124_CH_MAP_REG_SETUP (cfg) |
                      AD7124_CH_MAP_REG_AINP (ainp) |
                      AD7124_CH_MAP_REG_AINM (ainm) |
                      (enable ? AD7124_CH_MAP_REG_CH_ENABLE : 0);

    return writeRegister((AD7124_regIDs) ch);
  }
  return -1;
}

// Enable/Disable channel 
int Ad7124::enableChannel (uint8_t ch, bool enable){
  if (ch < 16) {        
    int ret;

    ch += Reg_Channel_0;
    
    ret = readRegister((AD7124_regIDs) ch);
    if (ret < 0) {
      return ret;
    }

    if (enable) {
      regs[ch].value |= AD7124_CH_MAP_REG_CH_ENABLE;
    }
    else {

      regs[ch].value &= ~AD7124_CH_MAP_REG_CH_ENABLE;
    }

    return writeRegister((AD7124_regIDs) ch);
  }
  return -1;
}

// Returns the setup number used by the channel 
int Ad7124::channelSetup(uint8_t ch){
  if (ch < 16) {    
    uint8_t setup;

    ch += Reg_Channel_0;    

    setup = (regs[ch].value >> 12) & 0x07;
    return setup;
  }
  return -1;
}

// Returns contents of status register 
int Ad7124::status() {  
  return readRegister(Reg_Status);
}

int Ad7124::currentChannel() {
  return regs[Reg_Status].value & 0x0F;
}

// Start a conversion on given channel 
int Ad7124::startSingleConversion (uint8_t ch) {

  if (ch < 16) {
    int ret;

    ret = enableChannel(ch, true);
    if (ret < 0) {
      return ret;
    }
    return setMode(AD7124_OpMode_SingleConv);
  }
  return -1;
}

// This is the old getData() that works if we are NOT useing data + status mode
// Returns positive raw ADC counts, or negative error code 
// int32_t Ad7124::getData(){
//   int ret;  
  
//   ret = readRegister(Reg_Data);
//   if (ret < 0) {
//     return ret; //Error
//   }

//   return regs[Reg_Data].value; //Legit value
// }



// Returns positive raw ADC counts, or negative error code 
// This version assumes the data + status mode is enabled. As long as testing
// checks out, that will be how the library will operate from now on -JJ 5-18-2021
int32_t Ad7124::getData(){
  int ret;

  // Temporary reg struct for data with extra byte to hold status bits
  Ad7124_Register Reg_DataAndStatus {0x02, 0x0000, 4, 2}; 

  // ret = waitForSpiReady (timeout); [commented out 8-26-21, may not be needed and looking for bottleneck in sample rate]
  // if (ret < 0) {
  //     return ret;
  // }
      
  ret = noCheckReadRegister(&Reg_DataAndStatus);
  if (ret < 0) {
      return ret;
  }

  regs[Reg_Status].value = Reg_DataAndStatus.value & 0xFF;
  regs[Reg_Data].value = (Reg_DataAndStatus.value >> 8) & 0x00FFFFFF;

    
  return regs[Reg_Data].value;
}

// Convert raw ADC data to volts 
double Ad7124::toVolts (long value, int gain, double vref, bool bipolar) {
  double voltage = (double) value;

  if (bipolar) {
    voltage = voltage / (double) 0x7FFFFFUL - 1;
  }
  else {
    voltage = voltage / (double) 0xFFFFFFUL;
  }

  voltage = voltage * vref / (double) gain;
  return voltage;
}

// Convert raw value from IC temperature sensor to degrees C 
double Ad7124::tempSensorRawToDegC(long value){
  //Conversion from datasheet https://www.analog.com/media/en/technical-documentation/data-sheets/AD7124-4.pdf
  return ((value - 0x800000)/13548.00)-272.5;
}



//// Private internal methods /////////////////////////////////////////////////

//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// UPDATE: readRegister and writeRegister have been updated to use the class regs 
// struct directly, eliminating the redundant pointer passing. The no-check versions
// are still using the pointer for now though
//!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!


// Reads the value of the specified register without checking if the chip is ready
int Ad7124::noCheckReadRegister(Ad7124_Register *reg){ 
    int ret           = 0;
    uint8_t buffer[8] = {0, 0, 0, 0, 0, 0, 0, 0};    
    uint8_t check8    = 0;
    uint8_t msgBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};

    
    //Check that we can read the register
    if ((!reg) || (reg->rw == AD7124_W)) {
        //Serial.println(reg->rw);
        return AD7124_INVALID_VAL;
    }

    /* Build the Command word */
    buffer[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD |
                AD7124_COMM_REG_RA (reg->addr);

    // Read data from the device 
    // NOTE: Arduino SPI transfer does not return an error code, so no check
    spiWriteAndRead(buffer, ( (crcEnabled != AD7124_DISABLE_CRC) ? reg->size + 1 : reg->size) + 1);
    
    
    /* Check the CRC */
    if (crcEnabled == AD7124_USE_CRC) {

        msgBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_RD | AD7124_COMM_REG_RA (reg->addr);
        
        for (int i = 1; i < reg->size + 2; ++i) {
            msgBuf[i] = buffer[i];
        }

        check8 = computeCRC8 (msgBuf, reg->size + 2);
    }

    if (check8 != 0) {
        /* readRegister checksum failed. */
        return AD7124_COMM_ERR;
    }

    /* Build the result */
    reg->value = 0;
    for (int i = 1; i < reg->size + 1; i++) {

        reg->value <<= 8;
        reg->value += buffer[i];
    }

    return ret;
}


// Writes the value of the specified register without checking if ready 
int Ad7124::noCheckWriteRegister(Ad7124_Register reg){
    int ret          = 0;
    int32_t regValue = 0;
    uint8_t wrBuf[8] = {0, 0, 0, 0, 0, 0, 0, 0};    
    uint8_t crc8     = 0;

    //Check that we can write to the register
    if (reg.rw == AD7124_R ) { 
        Serial.println(reg.rw);
        return AD7124_INVALID_VAL;
    }

    /* Build the Command word */
    wrBuf[0] = AD7124_COMM_REG_WEN | AD7124_COMM_REG_WR |
                AD7124_COMM_REG_RA (reg.addr);

    /* Fill the write buffer */
    regValue = reg.value;
    for (int i = 0; i < reg.size; i++) {
        wrBuf[reg.size - i] = regValue & 0xFF;
        regValue >>= 8;
    }

    /* Compute the CRC */
    if (crcEnabled != AD7124_DISABLE_CRC) {        
        crc8 = computeCRC8 (wrBuf, reg.size + 1);
        wrBuf[reg.size + 1] = crc8;
    }

    /* Write data to the device */
    spiWriteAndRead(wrBuf, (crcEnabled != AD7124_DISABLE_CRC) ? reg.size + 2 : reg.size + 1);    

    return 0; 
}

// Read a register, checks if ready first 
int32_t Ad7124::readRegister(AD7124_regIDs id){
    int ret;

    if (regs[id].addr != AD7124_ERR_REG && isReady) {        
        ret = waitForSpiReady (timeout);
        if (ret < 0) {
            return ret;
        }
    }    
    ret = noCheckReadRegister(&regs[id]); //THIS CAN'T BE RIGHT! If test above fails, we execute this anyway???

    return ret;    
}

// Write a register, checks if ready first 
int Ad7124::writeRegister(AD7124_regIDs id){
  int ret;
  
  ret = waitForSpiReady (timeout);
  if (ret < 0) {
      return ret;
  }
  
  ret = noCheckWriteRegister(regs[id]);

  return ret;
}

// Wait for device to be ready for read or write 
int Ad7124::waitForSpiReady (uint32_t timeout) {
    int  ret;
    bool ready = false;
    //bool timeout_en;
    
    int startMs = millis();

    while(1) {
        /* Read the value of the Error Register */        
        //ret = readRegister(Reg_Error);  //WTF!! This makes another call to waitForSpiReady internally, ITS TURTLES ALL THE WAY DOWN!!!! [8-26-21]
        ret = noCheckReadRegister(&regs[Reg_Error]); //Lets try this here instead [8-26-21]
        if (ret < 0) {
            return ret; // Problem, bail and forward the error
        }

        /* Check the SPI IGNORE Error bit in the Error Register */
        ready = (regs[Reg_Error].value & AD7124_ERR_REG_SPI_IGNORE_ERR) == 0;
        if(ready) {
            return ready; //Got it, we're done
        }

        if((millis() - startMs) > timeout){
          return AD7124_TIMEOUT; //Time out
        }
      
    }
    //Should never be here!
}

// Waits until power-on reset finishes 
int Ad7124::waitForPowerOn (uint32_t timeout) {
    int ret;
    bool powered_on = false;
    
    while(1) {
        ret = readRegister(Reg_Status);
        if (ret < 0) {
            return ret; // Problem, bail and forward the error
        }

        /* Check the POR_FLAG bit in the Status Register */
        powered_on = (regs[Reg_Status].value & AD7124_STATUS_REG_POR_FLAG) == 0;
        if(powered_on){
            return powered_on;
        }

        if (timeout) {
            delay (1); // <-- Should we do something better here? Only called on startup so probably okay.
            timeout--;
        }else{
            return AD7124_TIMEOUT;
        }
    } 
    //Should never be here!
}

// Waits until a new conversion result is available. 
int Ad7124::waitForConvReady (uint32_t timeout) {
  int ret;
  bool ready = false;
  
  int startMs = millis();

  while(1) {
    
    //ret = readRegister(Reg_Status);
    ret = noCheckReadRegister(&regs[Reg_Status]); //Try no check version here, if waiting for conversion, we should be ready for the read [8-26-21]
    if (ret < 0) {
      return ret;   // Problem, bail and forward the error
    }

    /* Check the RDY bit in the Status Register */
    ready = (regs[Reg_Status].value & AD7124_STATUS_REG_RDY) == 0;
    if(ready){
        return ready;
    }

    if((millis() - startMs) > timeout){
        return AD7124_TIMEOUT; //Time out
    }

  }
}


// Computes the CRC checksum for a data buffer.
// NOT USED YET
uint8_t Ad7124::computeCRC8(uint8_t *buffer, uint8_t size){        
    uint8_t crc;

    while(size){
        for(int i = 80;  i != 0; i >>= 1){
            if(((crc & 0x80) != 0) != ((*buffer & i) != 0)) { /* MSB of CRC register XOR input Bit from Data */
                    crc <<= 1;
                    crc ^= AD7124_CRC8_POLYNOMIAL_REPRESENTATION;
                } else {
                    crc <<= 1;
                }
        }
        buffer++;
        size--;
    }
    return crc;
}


// Writes and reads bytes in data buffer. 
// The received data is stored in the buffer in-place (the old data is replaced with the data received).
void Ad7124::spiWriteAndRead(uint8_t *data, uint8_t numBytes)
{
    // Serial.println("spiWriteAndRead ");
    // Serial.flush();

    SPI.beginTransaction(spiSettings);
    digitalWrite(cs, LOW);
    SPI.transfer(data, numBytes);
    digitalWrite(cs, HIGH);
    SPI.endTransaction();

}