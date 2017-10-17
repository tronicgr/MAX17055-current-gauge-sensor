/**************************************************************************/
/*! 
   	@file     Thanos_MAX17055.cpp
   	@author   Thanos Kontogiannis 
	@license  BSD License
	
	Driver for the MAX17055 current sensor		
	@section  HISTORY

   	v1.0 - First release
*/
/**************************************************************************/
#include "Arduino.h"
#include <Wire.h>
#include "Thanos_MAX17055.h"

/**************************************************************************/
/*! 
    @brief  Instantiates a new MAX17055 class
*/
/**************************************************************************/
Thanos_MAX17055::Thanos_MAX17055(uint8_t addr) {
  MAX17055_i2caddr = addr;
}

void Thanos_MAX17055::readRegister(byte reg, uint16_t *value) 
{
	byte MSB = 0;
	byte LSB = 0;
    Wire.beginTransmission(MAX17055_i2caddr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(MAX17055_i2caddr, 2);
	LSB = Wire.read();
	MSB = Wire.read();
    *value = (MSB << 8) | LSB;	
}

void Thanos_MAX17055::readbyteRegister(byte reg, byte &MSB, byte &LSB) {
	Wire.beginTransmission(MAX17055_i2caddr);
	Wire.write(reg);
	Wire.endTransmission();
	Wire.requestFrom(MAX17055_i2caddr, 2);
	LSB = Wire.read();
	MSB = Wire.read();
}

void Thanos_MAX17055::writeRegister(byte reg, uint16_t value) 
{
	byte MSB = 0;
	byte LSB = 0;
   	MSB = (value >> 8) & 0xFF;
	LSB = value & 0xFF;
	Wire.beginTransmission(MAX17055_i2caddr);
	Wire.write(reg);
	Wire.write(LSB);
	Wire.write(MSB);
	Wire.endTransmission();
}


/**************************************************************************/
/*! 
    @brief  Configures to MAX17055
			
*/
/**************************************************************************/
void Thanos_MAX17055::setConfigRegister(void)
{
  // Sets battery capacity to 2500mA and correct battery model
  // Set Battery Config registers 
	writeRegister(MAX17055_REG_Status_Reset, MAX17055_Status_Reset);
	writeRegister(MAX17055_REG_CONFIG_DesignCap, MAX17055_CONFIG_DesignCap);
	writeRegister(MAX17055_REG_CONFIG_FullCapRep, MAX17055_CONFIG_FullCapRep);
	writeRegister(MAX17055_REG_CONFIG_dQAcc, MAX17055_CONFIG_dQAcc);
	writeRegister(MAX17055_REG_CONFIG_dPAcc, MAX17055_CONFIG_dPAcc);
	writeRegister(MAX17055_REG_CONFIG_IChgTerm, MAX17055_CONFIG_IChgTerm);
	writeRegister(MAX17055_REG_CONFIG_VEmpty, MAX17055_CONFIG_VEmpty);
	writeRegister(MAX17055_REG_Command_HipCfg, MAX17055_Command_HipCfg_Exit1);
	writeRegister(MAX17055_REG_CONFIG_HipCfg, MAX17055_CONFIG_HipCfg);
	writeRegister(MAX17055_REG_Command_HipCfg, MAX17055_Command_HipCfg_Exit3);
	writeRegister(MAX17055_REG_CONFIG_Miscfg, MAX17055_CONFIG_Miscfg);
	writeRegister(MAX17055_REG_CONFIG_ModelCfg, MAX17055_CONFIG_ModelCfg);
	writeRegister(MAX17055_REG_CONFIG_RComp0, MAX17055_CONFIG_RComp0);
	writeRegister(MAX17055_REG_CONFIG_TempCo, MAX17055_CONFIG_TempCo);
	writeRegister(MAX17055_REG_Status_Reset, MAX17055_Status_Reset_POR);
	writeRegister(MAX17055_REG_CONFIG_HipCfg, MAX17055_CONFIG_HipCfg);
}

/**************************************************************************/
/*! 
    @brief  Setups the HW 
*/
/**************************************************************************/
void Thanos_MAX17055::begin(void) {
  Wire.begin();    
  // Set chip to large range config values to start
  setConfigRegister();
}

/**************************************************************************/
/*! 
    @brief  Gets the raw State Of Charge SOC (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getRepSOC_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_RepSOC, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getCurrent_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_CURRENT, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw Average current value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getAvgCurrent_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_AVGCURRENT, &value);
  return (int16_t)value;
}
 
/**************************************************************************/
/*! 
    @brief  Gets the raw Reported Capacity value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getRepCap_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_RepCap, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw Full Reported Capacity value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getFullRepCap_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_FullRepCap, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw Temperature value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getTemp_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_Temp, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw Time to Empty value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getTTE_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_TTE, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the raw Time to Full value (16-bit signed integer, so +-32767)
*/
/**************************************************************************/
int16_t Thanos_MAX17055::getTTF_raw() {
  uint16_t value;
  readRegister(MAX17055_REG_TTF, &value);
  return (int16_t)value;
}

/**************************************************************************/
/*! 
    @brief  Gets the current value in mA
*/
/**************************************************************************/
float Thanos_MAX17055::getCurrent_mA() {
  int16_t valueDec = getCurrent_raw();
  return valueDec * 0.15625;
  //return (32766 - valueDec) * 0.15625;
}

/**************************************************************************/
/*! 
    @brief  Gets the Average current value in mA
*/
/**************************************************************************/
float Thanos_MAX17055::getAvgCurrent_mA() {
  int16_t valueDec = getAvgCurrent_raw();
  return valueDec * 0.15625;
  //return (32766 - valueDec) * 0.15625;
}

/**************************************************************************/
/*! 
    @brief  Gets the Battery Voltage value in volts
*/
/**************************************************************************/

float Thanos_MAX17055::getVoltageCell() {
  int value;
  byte MSB = 0;
  byte LSB = 0;
  readbyteRegister(MAX17055_REG_VCELL, MSB, LSB);
  value =  ((MSB << 8) | LSB);
  value = ((value / 16) * 1.25);
  return value;
}

/**************************************************************************/
/*! 
    @brief  Gets the Battery SOC (State of Charge) value in Percent
*/
/**************************************************************************/
float Thanos_MAX17055::getRepSOC() {	
	int16_t valueDec = getRepSOC_raw();
  	return valueDec / 256 ;	
}

/**************************************************************************/
/*! 
    @brief  Gets the Battery Full Capacity value in mAh
*/
/**************************************************************************/

float Thanos_MAX17055::getFullRepCap() {
	int16_t valueDec = getFullRepCap_raw();
  	return valueDec;	
}


/**************************************************************************/
/*! 
    @brief  Gets the Battery Reported Capacity value in mAh
*/
/**************************************************************************/
float Thanos_MAX17055::getRepCap() {
	int16_t valueDec = getRepCap_raw();
  	return valueDec;	
}

/**************************************************************************/
/*! 
    @brief  Gets the Temperature value in Celcius
*/
/**************************************************************************/
float Thanos_MAX17055::getTemp() {
	
	int16_t valueDec = getTemp_raw();
  	return valueDec / 256.0 ;	
}

/*! 
    @brief  Gets the Time to Empty value in hours
*/
/**************************************************************************/
float Thanos_MAX17055::getTTE() {	
	int16_t valueDec = getTTE_raw();
  	return valueDec / 640.002 ;	
}

/*! 
    @brief  Gets the Time to Full value in hours
*/
/**************************************************************************/
float Thanos_MAX17055::getTTF() {	
	int16_t valueDec = getTTF_raw();
  	return valueDec / 640.002 ;	
}

void Thanos_MAX17055::reset() {
	writeRegister(MAX17055_REG_Status_Reset, MAX17055_Status_Reset);
}	

void Thanos_MAX17055::quickStart() {	
	writeRegister(MAX17055_REG_Status_Reset, MAX17055_Status_Reset);
	writeRegister(MAX17055_REG_CONFIG_Miscfg, MAX17055_CONFIG_Miscfg);
	writeRegister(MAX17055_REG_Status_Reset, MAX17055_Status_Reset_POR);
	writeRegister(MAX17055_REG_CONFIG_HipCfg, MAX17055_CONFIG_HipCfg);
}
