/*!
 * @file  DFRobot_RS01.cpp
 * @brief  Define the infrastructure DFRobot_RS01 class
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author   [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-07-06
 * @url   https://github.com/DFRobot/DFRobot_RS01
 */
#include "DFRobot_RS01.h"

DFRobot_RS01::DFRobot_RS01(uint8_t addr)
{
  basicInfo.modbusAddr = addr;   // RS01modbus communication address
  measurementConfig.startPosition = 0x00C8;   // Default measurement start position 200
  measurementConfig.stopPosition = 0x1770;   // Default measurement stop position 6000
  measurementConfig.initialThreshold = 0x0190;   // Default initial threshold 400
  measurementConfig.endThreshold = 0x0190;   // Default end threshold 400
  measurementConfig.moduleSensitivity = 0x0002;   // Default measurement sensitivity 2
  measurementConfig.comparisonOffset = 0x0000;   // Default comparison offset 0
}

int DFRobot_RS01::begin(Stream *_serial)
{
  if(basicInfo.modbusAddr > 0xF7){
   DBG("Invaild Device addr.");
  }

  _DFRobot_RTU = new DFRobot_RTU(_serial);   // Instantiate a modbus-RTU object for register reading and writing communication
  delay(1000);   // wait for 1s

  _DFRobot_RTU->setTimeoutTimeMs(500);   // Set the return message timeout to 500ms
  delay(100);

  uint16_t pid=0;
  if(0 != readData(RS01_PID_REG, &pid, 1))   // Judge whether the data bus is successful
  {
    DBG("ERR_DATA_BUS");
    return ERR_DATA_BUS;
  }

  DBG("real sensor pid=");DBG(pid,HEX);
  if(RS01_PID != pid)   // Judge whether the chip version matches
  {
    DBG("ERR_IC_VERSION");
    return ERR_IC_VERSION;
  }

  return NO_ERROR;
}

/***************** Sensor information reading ******************************/

int DFRobot_RS01::refreshBasicInfo(void)
{
  return readData(RS01_PID_REG, (uint16_t *)&basicInfo, 6);
}

int DFRobot_RS01::refreshMeasurementData(void)
{
  return readData(RS01_TARGETS_NUMBER, dataBuf, 11);
}

int DFRobot_RS01::refreshMeasurementConfig(void)
{
  return readData(MEASUREMENT_START_POSITION, (uint16_t *)&measurementConfig, 6);
}

/***************** Sensor basic information config ******************************/

void DFRobot_RS01::setADDR(uint16_t addr)
{
  if((0x0001 <= addr) && (0x00F7 >= addr))
  {
    uint8_t ret = writeData(RS01_ADDR_REG, &addr, 1);
    if(ret){
      DBG(ret);
    }else{
      basicInfo.modbusAddr = addr;
    }
  }
  delay(100);
}

void DFRobot_RS01::setBaudrateMode(eBaudrateMode_t mode)
{
  uint8_t ret = writeData(RS01_BAUDRATE_REG, &mode, 1);
  if(ret){
    DBG(ret);
  }else{
    basicInfo.baudrate = mode;
  }
  delay(100);
}


void DFRobot_RS01::setCheckbitStopbit(uint16_t mode)
{
  uint8_t ret = writeData(RS01_CHECKBIT_STOPBIT_REG, &mode, 1);
  if(ret){
    DBG(ret);
  }else{
    basicInfo.checkbit = (uint8_t)((mode & 0xFF00) >> 8);
    basicInfo.stopbit = (uint8_t)(mode & 0x00FF);
  }
  delay(100);
}

/***************** Sensor measurement parameters config ******************************/

void DFRobot_RS01::setAllMeasurementParameters(uint16_t startingPosition, uint16_t stopPosition,
                                                  uint16_t initialThreshold, uint16_t endThreshold,
                                                  uint16_t moduleSensitivity, uint16_t comparisonOffset)
{
  if(readData(MEASUREMENT_START_POSITION, (uint16_t *)&measurementConfig, 6)){
    DBG();
  }
  delay(100);

  if((0x0046 <= startingPosition) && (measurementConfig.stopPosition >= startingPosition))
  {
    measurementConfig.startPosition = startingPosition;   // Measurement start position set value
  }

  if((measurementConfig.startPosition <= stopPosition) && (0x19C8 >= stopPosition))
  {
    measurementConfig.stopPosition = stopPosition;   // Measurement stop position set value
  }

  if((0x0064 <= initialThreshold) && (0x2710 >= initialThreshold) && (0 < int16_t(initialThreshold + measurementConfig.comparisonOffset)))
  {
    measurementConfig.initialThreshold = initialThreshold;   // Initial threshold set value
  }

  if((0x0064 <= endThreshold) && (0x2710 >= endThreshold) && (0 < int16_t(endThreshold + measurementConfig.comparisonOffset)))
  {
    measurementConfig.endThreshold = endThreshold;   // End threshold set value
  }

  if(0x0004 >= moduleSensitivity)
  {
    measurementConfig.moduleSensitivity = moduleSensitivity;   // Module sensitivity set value
  }

  if((0 < int16_t(measurementConfig.initialThreshold + comparisonOffset)) && (0 < int16_t(comparisonOffset + measurementConfig.endThreshold)))
  {
    measurementConfig.comparisonOffset = comparisonOffset;   // Comparison offset set value
  }

  if(writeData(MEASUREMENT_START_POSITION, &measurementConfig, 6)){
    DBG();
  }
  delay(100);
}

void DFRobot_RS01::restoreFactorySetting(void)
{
  uint16_t value = 0x0000;   // To zero out the register is a soft reset
  if(writeData(RS01_RESET_FACTORY, &value, 1)){
    DBG();
  }
}

/************ Modbus-RTU interface init and read/write ***********/

uint8_t DFRobot_RS01::readData(uint16_t reg, uint16_t * pBuf, uint8_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }
  uint8_t ret = _DFRobot_RTU->readHoldingRegister(basicInfo.modbusAddr, reg, pBuf, size);
  if(ret){
    DBG(ret);
  }
  // delay(100);
  return ret;
}

uint8_t DFRobot_RS01::writeData(uint16_t reg, const void * pBuf, uint8_t size)
{
  if(NULL == pBuf)
  {
    DBG("pBuf ERROR!! : null pointer");
  }

  uint8_t ret = _DFRobot_RTU->writeHoldingRegister(basicInfo.modbusAddr, reg, (uint16_t *)pBuf, size);
  if(ret){
    DBG(ret);
  }
  // delay(100);
  return ret;
}
