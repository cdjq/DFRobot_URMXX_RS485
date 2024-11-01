/*!
 * @file  DFRobot_URMXX.h
 * @brief  Define infrastructure of DFRobot_URMXX class
 * @details  Get and configure the sensor basic information and measurement parameters, and the sensor measurement information
 * @copyright  Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author  [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2024-11-01
 * @url  https://github.com/DFRobot/DFRobot_URMXX
 */
#ifndef __DFROBOT_URMXX_H__
#define __DFROBOT_URMXX_H__

#include <Arduino.h>
#include <Stream.h>
#include <DFRobot_RTU.h>

#define ENABLE_DBG   //!< Open the macro and you can see the details of the program
#ifdef ENABLE_DBG
#define DBG(...) {Serial.print("[");Serial.print(__FUNCTION__); Serial.print("(): "); Serial.print(__LINE__); Serial.print(" ] "); Serial.println(__VA_ARGS__);}
#else
#define DBG(...)
#endif

/* Universal default value */
#define URMXX_MODULE_VID                uint16_t(0x3343)   ///< module VID, the default value is 0x3343 (represent manufacturer DFRobot)
#define URMXX_DEFAULT_ADDR              uint16_t(0x000F)   ///< URMXX default address
#define URMXX_DEFAULT_BAUDRATE          uint16_t(0x0003)   ///< URMXX default baud rate
#define URMXX_DEFAULT_PARITY_AND_STOP   uint16_t(0x0003)   ///< URMXX default parity bit and stop bit

/* URM08 default value */
#define URM08_MODULE_PID                uint16_t(0x0004)   ///< URM08-RS485 Waterproof Sonar Range Finder PID (SEN0246)
// #define URM08_DEFAULT_ADDR              uint16_t(0x000F)   ///< URM08 default address
// #define URM08_DEFAULT_BAUDRATE          uint16_t(0x0003)   ///< URM08 default baud rate
// #define URM08_DEFAULT_PARITY_AND_STOP   uint16_t(0x0003)   ///< URM08 default parity bit and stop bit

/* URM12 default value */
#define URM12_MODULE_PID                uint16_t(0x0001)   ///< URM12 ultrasonic sensor 1500cm RS485 PID (SEN0310)
// #define URM12_DEFAULT_ADDR              uint16_t(0x000F)   ///< URM12 default address
// #define URM12_DEFAULT_BAUDRATE          uint16_t(0x0003)   ///< URM12 default baud rate
// #define URM12_DEFAULT_PARITY_AND_STOP   uint16_t(0x0003)   ///< URM12 default parity bit and stop bit

/* URM14 default value */
#define URM14_MODULE_PID                uint16_t(0x0002)   ///< URM14 RS485 ultrasonic sensor PID (SEN0358)
// #define URM14_DEFAULT_ADDR              uint16_t(0x000F)   ///< URM14 default address
// #define URM14_DEFAULT_BAUDRATE          uint16_t(0x0003)   ///< URM14 default baud rate
// #define URM14_DEFAULT_PARITY_AND_STOP   uint16_t(0x0003)   ///< URM14 default parity bit and stop bit

/* URM15 default value */
#define URM15_MODULE_PID                uint16_t(0x0003)   ///< URM15 RS485 ultrasonic sensor PID (SEN0519)
// #define URM15_DEFAULT_ADDR              uint16_t(0x000F)   ///< URM15 default address
// #define URM15_DEFAULT_BAUDRATE          uint16_t(0x0003)   ///< URM15 default baud rate
// #define URM15_DEFAULT_PARITY_AND_STOP   uint16_t(0x0003)   ///< URM15 default parity bit and stop bit


/* URMXX input register address for basic information */
#define URMXX_VID_REG               uint16_t(0x0000)   ///< module VID memory register, the default value is 0x3343 (represent manufacturer DFRobot)
#define URMXX_PID_REG               uint16_t(0x0001)   ///< module PID memory register, corresponding product model
#define URMXX_HARD_V_REG            uint16_t(0x0002)   ///< memory register for hardware revision number:0x1000 represents V1.0.0.0
#define URMXX_SOFT_V_REG            uint16_t(0x0003)   ///< memory register for software revision number:0x1000 represents V1.0.0.0
#define URMXX_DISTANCE_REG          uint16_t(0x0004)   ///< URMXX distance register
#define URMXX_TEMP_REG              uint16_t(0x0005)   ///< URMXX temperature register

/* URM08 input register address of measurement data */
// #define URM08_DISTANCE_REG          uint16_t(0x0004)   ///< URM08 distance register (U: 1 cm, R: 35~550 cm, F: 38~42KHz, DC: 6~12 V, T: 1000/10 ms)
// #define URM08_TEMP_REG              uint16_t(0x0005)   ///< URM08 temperature register (U: 0.1 °C, R: -10~70 °C)

/* URM12 input register address of measurement data */
// #define URM12_DISTANCE_REG          uint16_t(0x0004)   ///< URM12 distance register (U: 1 cm, R: 70~1500 cm, F: 38~42KHz, DC: 9~24 V, T: 1000/3 ms)
// #define URM12_TEMP_REG              uint16_t(0x0005)   ///< URM12 temperature register (U: 0.1 °C, R: -10~70 °C)

/* URM14 input register address of measurement data */
// #define URM14_DISTANCE_REG          uint16_t(0x0004)   ///< URM14 distance register (U: 0.1 mm, R: 100~1500 mm, F: 200±4KHz, DC: 7~15 V, T: 1000/30 ms)
// #define URM14_TEMP_REG              uint16_t(0x0005)   ///< URM14 temperature register (U: 0.1 °C, R: -20~80 °C)
#define URM14_NOISE_REG              uint16_t(0x0006)   ///< URM14 Power supply noise level register (R: 0-10)

/* URM15 input register address of measurement data */
// #define URM15_DISTANCE_REG          uint16_t(0x0004)   ///< URM15 distance register (U: 0.1 cm, R: 30~500 cm, F: 75±2KHz, DC: 5~12 V, T: 1000/10ms)
// #define URM15_TEMP_REG              uint16_t(0x0005)   ///< URM15 temperature register (U: 0.1 °C, R: -10~70 °C)


/* URMXX holding register address for communication configuration information */
#define URMXX_ADDR_REG              uint16_t(0x0000)   ///< module check bit and stop bit memory register, the default value is 0x0001
#define URMXX_BAUDRATE_REG          uint16_t(0x0001)   ///< memory register for serial baud rate, the default value is 0x0003
#define URMXX_PARITY_AND_STOP_REG   uint16_t(0x0002)   ///< memory register for serial port parity bit and stop bit, the default value is 0x0003

/* URM12 holding register address of configuration information */
#define URM12_EXTERNAL_TEMP_REG     uint16_t(0x0003)   ///< URM12 external temp register
#define URM12_CR_REG                uint16_t(0x0004)   ///< URM12 CR register

/* URM14 holding register address of configuration information */
// #define URM14_EXTERNAL_TEMP_REG     uint16_t(0x0003)   ///< URM14 external temp register
// #define URM14_CR_REG                uint16_t(0x0004)   ///< URM14 CR register

/* URM15 holding register address of configuration information */
// #define URM15_EXTERNAL_TEMP_REG     uint16_t(0x0003)   ///< URM15 external temp register
// #define URM15_CR_REG                uint16_t(0x0004)   ///< URM15 CR register


class DFRobot_URMXX
{
public:
#define NO_ERROR           0     ///< No error
#define ERR_DATA_BUS      (-1)   ///< data bus error
#define ERR_IC_VERSION    (-2)   ///< the chip version isn't suitable
  /**
   * @struct sBasicInfo_t
   * @brief device information structure in modbus mode
   */
  typedef struct
  {
    uint16_t VID;   /**< module VID, the default value is 0x3343 (represent manufacturer DFRobot) */
    uint16_t PID;   /**< module PID, corresponding product model */
    uint16_t HARD_V;   /**< hardware revision number: 0x1000 represents V1.0.0.0 */
    uint16_t SOFT_V;   /**< software revision number: 0x1000 represents V1.0.0.0 */
    uint16_t modbusAddr;   /**< module communication address, module device address(1~247) */
    uint16_t baudrate;   /**< module baud rate */
    uint8_t stopbit;   /**< low 8 bit: stop bit: 0.5bit; 1bit; 1.5bit; 2bit */
    uint8_t checkbit;   /**< high 8 bit: check bit: 0 represents none; 1 represents even; 2 represents odd */
  }sBasicInfo_t;

  /**
   * @enum  eBaudrateMode_t
   * @brief Available baud rate for the module
   */
  typedef enum
  {
    // eBaudrate9600Default = 0x0000,
    eBaudrate2400 = 0x0001,
    eBaudrate4800 = 0x0002,
    eBaudrate9600 = 0x0003,
    eBaudrate14400 = 0x0004,
    eBaudrate19200 = 0x0005,
    eBaudrate38400 = 0x0006,
    eBaudrate57600 = 0x0007,
    eBaudrate115200 = 0x0008,
    eBaudrate_1000000 = 0x0009,
  }eBaudrateMode_t;

  /**
   * @enum  eCheckBitMode_t
   * @brief  Available check bit mode for the module
   */
  typedef enum
  {
    eCheckBitNone = 0x0000 << 8,
    eCheckBitEven = 0x0001 << 8,
    eCheckBitOdd = 0x0002 << 8,
  }eCheckBitMode_t;

  /**
   * @enum  eStopBitMode_t
   * @brief Available stop bit mode for the module
   */
  typedef enum
  {
    eStopBit0p5 = 0x0000,
    eStopBit1 = 0x0001,
    eStopBit1p5 = 0x0002,
    eStopBit2 = 0x0003,
  }eStopBitMode_t;

public:
  /**
   * @struct sURMXXConfig_t
   * @brief urmXX control register
   */
  typedef struct
  {
    uint8_t tempCompSource : 1;   /**< 0: Use onboard temperature compensation,  1: Use external temperature compensation work
                                      (requires user to write temperature data to external temperature compensation data register) */
    uint8_t tempCompMode : 1;   /**< 0: enable temperature compensation,  1: disable temperature compensation */
    uint8_t autoMeasureMode : 1; /**< 0: auto measuring distance,  1: passive measuring distance */
    uint8_t trigPasvMeasure : 1; /**< In passive mode, write 1 to the bit and the sensor will complete a distance measurement (about 300ms). */
    uint8_t reserved : 4; /**< reserved bit */
  } __attribute__((packed)) sURMXXConfig_t;

  /**
   * @enum  eTempCompSource_t
   * @brief  Temperature compensation source selection
   */
  typedef enum
  {
    eInternalTemp = 0,   /**< Use onboard temperature compensation */
    eExternalTemp,   /**< Use external temperature compensation */
  }eTempCompSource_t;

  /**
   * @enum  eTempCompMode_t
   * @brief  whether to enable temperature compensation or not
   */
  typedef enum
  {
    eTempCompEN = 0,   /**< enable temperature compensation */
    eTempCompDIS = 1,   /**< disable temperature compensation */
  }eTempCompMode_t;

  /**
   * @enum  eMeasureMode_t
   * @brief  distance measurement model
   */
  typedef enum
  {
    eAutoMeasureMode = 0,   /**< auto measuring distance */
    ePassiveMeasureMode = 1,   /**< passive measuring distance */
    eTrigPassiveMeasure = 1,   /**< In passive mode, write 1 to the bit and the sensor will complete a distance measurement (about 300ms). */
  }eMeasureMode_t;

public:
  /**
   * @fn DFRobot_URMXX
   * @brief constructor
   * @param addr RS485 communication device address
   * @return None
   */
  DFRobot_URMXX(uint8_t addr);

  /**
   * @fn begin
   * @brief init function
   * @param _serial serial ports for communication, supporting hard and soft serial ports
   * @return int type, means returning initialization status
   * @retval 0 NO_ERROR
   * @retval -1 ERR_DATA_BUS
   * @retval -2 ERR_IC_VERSION
   */
  int begin(Stream* _serial);

  /***************** Sensor information reading ******************************/

  /**
   * @fn getDistanceRaw
   * @brief Get distance measurement data
   * @return Returns 16 bits of distance data
   * @n URM08 : U: 1 cm, R: 35~550 cm, F: 38~42KHz, DC: 6~12 V, T: 1000/10 ms
   * @n URM12 : U: 1 cm, R: 70~1500 cm, F: 38~42KHz, DC: 9~24 V, T: 1000/3 ms
   * @n URM14 : U: 0.1 mm, R: 100~1500 mm, F: 200±4KHz, DC: 7~15 V, T: 1000/30 ms
   * @n URM15 : U: 0.1 cm, R: 30~500 cm, F: 75±2KHz, DC: 5~12 V, T: 1000/10ms
   */
  uint16_t getDistanceRaw(void);

  /**
   * @fn getTemperatureC
   * @brief Obtain temperature measurement data
   * @return Returns floating point temperature data in degrees Celsius
   * @note Unit: 0.1 °C, Range: -10~70 °C, Deviation: ±1 °C
   */
  float getTemperatureC(void);

  /**
   * @fn refreshBasicInfo
   * @brief Get and store basic information
   * @details After obtaining it, you can view it using the structure variable basicInfo
   *    typedef struct
   *    {
   *      uint16_t VID;   // module VID, the default value is 0x3343 (represent manufacturer DFRobot)
   *      uint16_t PID;   // module PID, corresponding product model
   *      uint16_t HARD_V;   // hardware revision number: 0x1000 represents V1.0.0.0
   *      uint16_t SOFT_V;   // software revision number: 0x1000 represents V1.0.0.0
   *      uint16_t modbusAddr;   // module communication address, module device address(1~247)
   *      uint16_t baudrate;   // module baud rate, the default value is 0x0009(1000000)
   *      uint8_t stopbit;   // stop bit: 0.5bit; 1bit; 1.5bit; 2bit
   *      uint8_t checkbit;   // check bit: 0 represents none; 1 represents even; 2 represents odd
   *    }sBasicInfo_t;
   * @return uint8_t, Exception code:
   * @retval   0 : sucess.
   * @retval   1 or eRTU_EXCEPTION_ILLEGAL_FUNCTION : Illegal function.
   * @retval   2 or eRTU_EXCEPTION_ILLEGAL_DATA_ADDRESS: Illegal data address.
   * @retval   3 or eRTU_EXCEPTION_ILLEGAL_DATA_VALUE:  Illegal data value.
   * @retval   4 or eRTU_EXCEPTION_SLAVE_FAILURE:  Slave failure.
   * @retval   8 or eRTU_EXCEPTION_CRC_ERROR:  CRC check error.
   * @retval   9 or eRTU_RECV_ERROR:  Receive packet error.
   * @retval   10 or eRTU_MEMORY_ERROR: Memory error.
   * @retval   11 or eRTU_ID_ERROR: Broadcasr address or error ID
   */
  uint8_t refreshBasicInfo(void);

  /***************** Sensor basic information config ******************************/

  /**
   * @fn setADDR
   * @brief Set the module communication address
   * @param addr Device address to be set, (1~247 is 0x0001~0x00F7)
   * @return None
   */
  void setADDR(uint16_t addr);

  /**
   * @fn setBaudrateMode
   * @brief Set the module baud rate, the setting takes effect after power fail and restart
   * @param mode The baud rate to be set:
   * @n       eBaudrate2400---2400, eBaudrate4800---4800, eBaudrate9600---9600,
   * @n       eBaudrate14400---14400, eBaudrate19200---19200, eBaudrate38400---38400,
   * @n       eBaudrate57600---57600, eBaudrate115200---115200, eBaudrate_1000000---1000000
   * @return None
   */
  void setBaudrateMode(eBaudrateMode_t mode);

  /**
   * @fn setCheckbitStopbit
   * @brief set check bit and stop bit of the module
   * @param mode the mode to be set, perform OR operation on the following to get mode:
   * @n       check bit:
   * @n             eCheckBitNone
   * @n             eCheckBitEven
   * @n             eCheckBitOdd
   * @n       stop bit:
   * @n             eStopBit1
   * @n             eStopBit2
   * @return None
   */
  void setCheckbitStopbit(uint16_t mode);

  /* Implemented in subclasses of special signal modules */
  virtual void setExternalTemperatureC(float temp)
  {
    (void)temp;
  };
  virtual float getExternalTemperatureC(void)
  {
    return 0.0f;
  };
  virtual void setControlRegister(sURMXXConfig_t mode)
  {
    (void)mode;
  };
  virtual sURMXXConfig_t getControlRegister(void)
  {
    return sURMXXConfig_t{};
  };
  virtual uint8_t getNoiseLevel(void)
  {
    return 0;
  };

public:
  /* variable for storing the information obtained by users */
  sBasicInfo_t basicInfo;

private:
  /* private variables */
};

class DFRobot_URM08 :public DFRobot_URMXX
{
public:
  /**
   * @fn DFRobot_URM08
   * @brief constructor
   * @param addr RS485 communication device address
   * @return None
   */
  DFRobot_URM08(uint8_t addr = URMXX_DEFAULT_ADDR);

private:

};

class DFRobot_URM12 :public DFRobot_URMXX
{
public:
  /**
   * @fn DFRobot_URM12
   * @brief constructor
   * @param addr RS485 communication device address
   * @return None
   */
  DFRobot_URM12(uint8_t addr = URMXX_DEFAULT_ADDR);

  /**
   * @fn setExternalTemperatureC
   * @brief The user writes the temperature for the external compensation function
   * @return None
   */
  virtual void setExternalTemperatureC(float temp);

  /**
   * @fn getExternalTemperatureC
   * @brief Gets the external compensation temperature of the current write
   * @return Returns floating point temperature data in degrees Celsius
   */
  virtual float getExternalTemperatureC(void);

  /**
   * @fn setControlRegister
   * @brief Configuration control register
   * @param mode sURMXXConfig_t:
   * @n       mode.tempCompSource : eInternalTemp: Use onboard temperature compensation,  eExternalTemp: Use external temperature compensation work
   * @n                             (requires user to write temperature data to external temperature compensation data register)
   * @n       mode.tempCompMode : eTempCompEN: enable temperature compensation,  eTempCompDIS: disable temperature compensation
   * @n       mode.autoMeasureMode : eAutoMeasureMode: auto measuring distance,  ePassiveMeasureMode: passive measuring distance
   * @n       mode.trigPasvMeasure : In passive mode, write 1 to the bit and the sensor will complete a distance measurement (about 300ms).
   * @n       mode.reserved : reserved bit
   * @return None
   */
  virtual void setControlRegister(sURMXXConfig_t mode);

  /**
   * @fn getControlRegister
   * @brief Gets the current control register value
   * @return sURMXXConfig_t:
   * @n       return.tempCompSource : eInternalTemp: Use onboard temperature compensation,  eExternalTemp: Use external temperature compensation work
   * @n                               (requires user to write temperature data to external temperature compensation data register)
   * @n       return.tempCompMode : eTempCompEN: enable temperature compensation,  eTempCompDIS: disable temperature compensation
   * @n       return.autoMeasureMode : eAutoMeasureMode: auto measuring distance,  ePassiveMeasureMode: passive measuring distance
   * @n       return.trigPasvMeasure : In passive mode, write 1(eTrigPassiveMeasure) to the bit and the sensor will complete a distance measurement (about 300ms).
   * @n       return.reserved : reserved bit
   */
  virtual sURMXXConfig_t getControlRegister(void);
private:

};

class DFRobot_URM14 :public DFRobot_URM12
{
public:
  /**
   * @fn DFRobot_URM14
   * @brief constructor
   * @param addr RS485 communication device address
   * @return None
   */
  DFRobot_URM14(uint8_t addr = URMXX_DEFAULT_ADDR);

  /**
   * @fn getNoiseLevel
   * @brief Get the power supply noise level
   * @details This parameter reflects the degree of influence of the power supply and environment on the sensor.
   * @n The smaller the noise level, the more accurate the distance value obtained by the sensor.
   * @return uint8_t 0x00-0x0A corresponds to noise levels 0-10.
   */
  virtual uint8_t getNoiseLevel(void);

private:

};

class DFRobot_URM15 :public DFRobot_URM12
{
public:
  /**
   * @fn DFRobot_URM15
   * @brief constructor
   * @param addr RS485 communication device address
   * @return None
   */
  DFRobot_URM15(uint8_t addr = URMXX_DEFAULT_ADDR);

private:

};

#endif
