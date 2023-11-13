/*!
 * @file  DFRobot_URMXX.cpp
 * @brief  Define the infrastructure DFRobot_URMXX class
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author   [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2021-07-06
 * @url   https://github.com/DFRobot/DFRobot_URMXX
 */
#include "DFRobot_URMXX.h"

DFRobot_RTU* pDFRobot_RTU;   // the pointer to RS485 communication mode instance

DFRobot_URMXX::DFRobot_URMXX(uint8_t addr)
{
  basicInfo.modbusAddr = addr;   // URMXXmodbus communication address
}

int DFRobot_URMXX::begin(Stream* _serial)
{
  if (basicInfo.modbusAddr > 0xF7) {
    DBG("Invaild Device addr.");
  }

  pDFRobot_RTU = new DFRobot_RTU(_serial);   // Instantiate a modbus-RTU object for register reading and writing communication
  delay(100);   // wait for 100 ms
  pDFRobot_RTU->setTimeoutTimeMs(500);   // Set the return message timeout to 500ms
  delay(100);

  uint16_t pid = 0;
  uint8_t ret = pDFRobot_RTU->readInputRegister(basicInfo.modbusAddr, URMXX_PID_REG, &pid, 1);
  if (0 != ret) {   // Judge whether the data bus is successful
    DBG("ERR_DATA_BUS"); DBG(ret);
    pDFRobot_RTU->~DFRobot_RTU();
    pDFRobot_RTU = NULL;
    return ERR_DATA_BUS;
  }

  DBG("real sensor pid="); DBG(pid, HEX);
  DBG("basicInfo.PID="); DBG(basicInfo.PID, HEX);
  if (pid != basicInfo.PID) {   // Judge whether the chip version matches
    DBG("ERR_IC_VERSION");
    pDFRobot_RTU->~DFRobot_RTU();
    pDFRobot_RTU = NULL;
    return ERR_IC_VERSION;
  }

  // If it is not urm8, then the control register needs to be initialized
  if (URM08_MODULE_PID != basicInfo.PID) {
    sURMXXConfig_t configMode = {
      .tempCompSource = eInternalTemp,
      .tempCompMode = eTempCompEN,
      .autoMeasureMode = eAutoMeasureMode,
      .trigPasvMeasure = 0,
      .reserved = 0,
    };
    setControlRegister(configMode);
  }

  return NO_ERROR;
}

/***************** Sensor information reading ******************************/

uint16_t DFRobot_URMXX::getDistanceRaw(void)
{
  uint16_t distance = 0;
  uint8_t ret = pDFRobot_RTU->readInputRegister(basicInfo.modbusAddr, URMXX_DISTANCE_REG, &distance, 1);
  if (ret) {
    DBG(ret);
    distance = 0;
  }
  delay(10);
  return distance;
}

float DFRobot_URMXX::getTemperatureC(void)
{
  int16_t temp = 0;
  uint8_t ret = pDFRobot_RTU->readInputRegister(basicInfo.modbusAddr, URMXX_TEMP_REG, (uint16_t *)&temp, 1);
  if (ret) {
    DBG(ret);
    temp = 0;
  }
  delay(10);
  return (float)temp / 10;
}

uint8_t DFRobot_URMXX::refreshBasicInfo(void)
{
  uint8_t ret = 0;
  ret = pDFRobot_RTU->readInputRegister(basicInfo.modbusAddr, URMXX_VID_REG, (uint16_t *)&basicInfo, 4);
  if (!ret) {
    ret = pDFRobot_RTU->readHoldingRegister(basicInfo.modbusAddr, URMXX_ADDR_REG, (uint16_t *)&basicInfo.modbusAddr, 3);
  }
  delay(10);
  return ret;
}

/***************** Sensor basic information config ******************************/

void DFRobot_URMXX::setADDR(uint16_t addr)
{
  if ((0x0001 <= addr) && (0x00F7 >= addr)) {
    uint8_t ret = pDFRobot_RTU->writeHoldingRegister(basicInfo.modbusAddr, URMXX_ADDR_REG, &addr, 1);
    if (ret) {   // Ignore the error because the address change takes effect immediately
      DBG(ret);
    }
    basicInfo.modbusAddr = addr;
  }
  delay(100);
}

void DFRobot_URMXX::setBaudrateMode(eBaudrateMode_t mode)
{
  uint8_t ret = pDFRobot_RTU->writeHoldingRegister(basicInfo.modbusAddr, URMXX_BAUDRATE_REG, (uint16_t *)&mode, 1);
  if (ret) {
    DBG(ret);
  } else {
    basicInfo.baudrate = mode;
  }
  delay(100);
}

void DFRobot_URMXX::setCheckbitStopbit(uint16_t mode)
{
  uint8_t ret = pDFRobot_RTU->writeHoldingRegister(basicInfo.modbusAddr, URMXX_PARITY_AND_STOP_REG, &mode, 1);
  if (ret) {
    DBG(ret);
  } else {
    basicInfo.checkbit = (uint8_t)((mode & 0xFF00) >> 8);
    basicInfo.stopbit = (uint8_t)(mode & 0x00FF);
  }
  delay(100);
}

/***************** Subclasses of different models ******************************/

/***************** URM08 ******************************/

DFRobot_URM08::DFRobot_URM08(uint8_t addr)
  :DFRobot_URMXX(addr)
{
  basicInfo.PID = URM08_MODULE_PID;   // URMXXmodbus communication address
}

/***************** URM12 ******************************/

DFRobot_URM12::DFRobot_URM12(uint8_t addr)
  :DFRobot_URMXX(addr)
{
  basicInfo.PID = URM12_MODULE_PID;   // URMXXmodbus communication address
}

void DFRobot_URM12::setExternalTemperatureC(float temp)
{
  uint16_t data = (int16_t)(temp * 10);
  uint8_t ret = pDFRobot_RTU->writeHoldingRegister(basicInfo.modbusAddr, URM12_EXTERNAL_TEMP_REG, (uint16_t *)&data, 1);
  if (ret) {
    DBG(ret);
  }
  delay(10);
}

float DFRobot_URM12::getExternalTemperatureC(void)
{
  uint16_t temp = 0;
  uint8_t ret = pDFRobot_RTU->readHoldingRegister(basicInfo.modbusAddr, URM12_EXTERNAL_TEMP_REG, (uint16_t*)&temp, 1);
  if (ret) {
    DBG(ret);
  }
  delay(10);
  return ((int16_t)temp) / 10.0;
}

void DFRobot_URM12::setControlRegister(sURMXXConfig_t mode)
{
  uint8_t ret = pDFRobot_RTU->writeHoldingRegister(basicInfo.modbusAddr, URM12_CR_REG, (uint16_t *)&mode, 1);
  if (ret) {
    DBG(ret);
  }
  delay(10);
}

DFRobot_URM12::sURMXXConfig_t DFRobot_URM12::getControlRegister(void)
{
  sURMXXConfig_t mode;
  uint8_t ret = pDFRobot_RTU->readHoldingRegister(basicInfo.modbusAddr, URM12_CR_REG, (uint16_t*)&mode, 1);
  if (ret) {
    DBG(ret);
  }
  delay(10);
  return mode;
}

/***************** URM14 ******************************/

DFRobot_URM14::DFRobot_URM14(uint8_t addr)
  :DFRobot_URM12(addr)
{
  basicInfo.PID = URM14_MODULE_PID;   // URMXXmodbus communication address
}

uint8_t DFRobot_URM14::getNoiseLevel(void)
{
  uint16_t noise = 0;
  uint8_t ret = pDFRobot_RTU->readInputRegister(basicInfo.modbusAddr, URM14_NOISE_REG, &noise, 1);
  if (ret) {
    DBG(ret);
    noise = 10;
  }
  delay(10);
  return (uint8_t)noise;
}

/***************** URM15 ******************************/

DFRobot_URM15::DFRobot_URM15(uint8_t addr)
  :DFRobot_URM12(addr)
{
  basicInfo.PID = URM15_MODULE_PID;   // URMXXmodbus communication address
}
