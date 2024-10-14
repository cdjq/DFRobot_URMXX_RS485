/*!
 * @file  getModuleInfo.ino
 * @brief  Get basic module information and measurement data
 * @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author   [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2023-10-19
 * @url   https://github.com/DFRobot/DFRobot_URMXX
 */
#include <DFRobot_URMXX.h>

 /* ------------------------------------------------------------------------------------------------
 *   board   |    MCU      | Leonardo/Mega2560/M0 |   UNO    | ESP8266 | ESP32 |  microbit  |   m0  |
 *    VCC    |   3.3V/5V   |        VCC           |   VCC    |   VCC   |  VCC  |     X      |  vcc  |
 *    GND    |     GND     |        GND           |   GND    |   GND   |  GND  |     X      |  gnd  |
 *    RX     |     TX      |     Serial1 TX1      |    3     |   5/D6  |  D2   |     X      |  tx1  |
 *    TX     |     RX      |     Serial1 RX1      |    2     |   4/D7  |  D3   |     X      |  rx1  |
 * -------------------------------------------------------------------------------------------------*/
#if defined(ARDUINO_AVR_UNO) || defined(ESP8266)
#include "SoftwareSerial.h"
SoftwareSerial urmSerial(/*rx =*/2, /*tx =*/3);
#else
#define urmSerial Serial1
#endif
char data[100];
 /**
  * DFRobot_URMXX constructor
  * Depending on the sensor used, open the corresponding macro
  * addr: modbus slave address(range1~247)or broadcast address(0x00)
  * If it's configured a broadcast address, send a broadcast packet, and all slaves on bus will process it but not respond
  */
//DFRobot_URM08 sensor(/*addr =*/URMXX_DEFAULT_ADDR);
 DFRobot_URM12 sensor(/*addr =*/URMXX_DEFAULT_ADDR);
// DFRobot_URM14 sensor(/*addr =*/URMXX_DEFAULT_ADDR);
// DFRobot_URM15 sensor(/*addr =*/URMXX_DEFAULT_ADDR);

void setup(void)
{
  Serial.begin(115200);
#if (defined ESP32)
  urmSerial.begin(9600, SERIAL_8N2, /*rx =*/D3, /*tx =*/D2);
#else
  urmSerial.begin(9600);
#endif

  /**
   * Init function
   * _serial Serial ports for communication, supporting hard and soft serial ports
   * returning 0 means reading succeeds
   */
  while (NO_ERROR != sensor.begin(/*s =*/&urmSerial)) {
    Serial.println("Communication with device failed, please check connection");
    delay(3000);
  }
  delay(3000);
  Serial.println("Begin ok!\n");
  
  Serial.println("\n-----------------read module basic information---------------------");
  /**
   * retrieve the basic information from the sensor and buffer it into the basicInfo structure that stores information:
   * basicInfo structure members:
   *     VID: module VID
   *     PID: module PID
   *     HARD_V: hardware revision number
   *     SOFT_V: software revision number
   *     modbusAddr: module communication address
   *     baudrate: module baud rate
   *     stopbit: stop bit
   *     checkbit: check bit
   * returning 0 means reading succeeds
   */
  if (0 == sensor.refreshBasicInfo()) {
    /* Module VID, the default value is 0x3343(represent manufacturer is DFRobot) */
    
    sprintf(data,"VID: 0x%04X\n",sensor.basicInfo.VID);
    Serial.println(data);

    /* Module PID, corresponding product model */

    sprintf(data,"PID: 0x%04X\n",sensor.basicInfo.PID);
    Serial.println(data);

    /* hardware revision number: 0x1000 represents V1.0.0.0 */
    sprintf(data,"HARD_V: 0x%04X\n",sensor.basicInfo.HARD_V);
    Serial.println(data);

    /* software revision number: 0x1000 represents V1.0.0.0 */
    sprintf(data,"SOFT_V: 0x%04X\n",sensor.basicInfo.SOFT_V);
    Serial.println(data);

    /* Module communication address, module device address(1~247) */
    Serial.print("communication address: ");
    Serial.println(sensor.basicInfo.modbusAddr);

    /* Module baud rate, the default value is 0x0003:
     * 0x0001---2400  0x0002---4800  0x0003---9600  0x0004---14400  0x0005---19200
     * 0x0006---38400  0x0007---57600  0x0008---115200  0x0009---1000000 */
  
    sprintf(data,"baudrate: 0x%04X\n",sensor.basicInfo.baudrate);
    Serial.println(data);
    
    /* Module check bit and stop bit, the default value is 0x0003
     * stop bit: 0.5bit; 1bit; 1.5bit; 2bit
     * check bit: 0 is none; 1 is even; 2 is odd */
    Serial.print("stop bit: ");
    Serial.println((sensor.basicInfo.stopbit + 1) / 2.0);
    Serial.print("check bit: ");
    Serial.println(sensor.basicInfo.checkbit);

  } else {
    Serial.println("Failed to read basic information!!!");
  }

  if (URM08_MODULE_PID != sensor.basicInfo.PID) {
    Serial.println("-----------------read module configuration information--------------------");
    /**
     *  Gets the external compensation temperature of the current write
     *  Returns floating point temperature data in degrees Celsius
     */
    float ExtTempC = sensor.getExternalTemperatureC();
    Serial.print("External temperature : ");  Serial.print(ExtTempC);  Serial.println(" °C");

    /**
     *  Gets the current control register value
     *  sURMXXConfig_t:
     *    configRet.tempCompSource : eInternalTemp: use onboard temperature compensation,  eExternalTemp: use external temperature compensation work
     *                            (requires user to write temperature data to external temperature compensation data register)
     *    configRet.tempCompMode : eTempCompEN: enable temperature compensation,  eTempCompDIS: disable temperature compensation
     *    configRet.autoMeasureMode : eAutoMeasureMode: auto measuring distance,  ePassiveMeasureMode: passive measuring distance
     *    configRet.trigPasvMeasure : In passive mode, write 1 to the bit and the sensor will complete a distance measurement (about 300ms).
     *    configRet.reserved : reserved bit
     */
    DFRobot_URMXX::sURMXXConfig_t configRet = sensor.getControlRegister();
    if (sensor.eInternalTemp == configRet.tempCompSource) {
      Serial.println("Use onboard temperature compensation.");
    } else {
      Serial.println("Use external temperature compensation work.");
    }
    if (sensor.eTempCompEN == configRet.tempCompMode) {
      Serial.println("Enable temperature compensation.");
    } else {
      Serial.println("Disable temperature compensation.");
    }
    if (sensor.eAutoMeasureMode == configRet.autoMeasureMode) {
      Serial.println("Auto measuring distance.");
    } else {
      Serial.println("Passive measuring distance.");
    }
  }

}

void loop()
{
  Serial.println("\n-----------------read module measurement data--------------------");
  /**
   * Get distance measurement data
   * Returns 16 bits of distance data
   *   URM08 : U: 1 cm, R: 35~550 cm, F: 38~42KHz, DC: 6~12 V, T: 1000/10 ms
   *   URM12 : U: 1 cm, R: 70~1500 cm, F: 38~42KHz, DC: 9~24 V, T: 1000/3 ms
   *   URM14 : U: 0.1 mm, R: 100~1500 mm, F: 200±4KHz, DC: 7~15 V, T: 1000/30 ms
   *   URM15 : U: 0.1 cm, R: 30~500 cm, F: 75±2KHz, DC: 5~12 V, T: 1000/10ms
   */
  uint16_t distanceRaw = sensor.getDistanceRaw();
  Serial.print("Distance : ");
  switch (sensor.basicInfo.PID) {
  case URM08_MODULE_PID:
    Serial.print(distanceRaw * 1); Serial.println(" cm");
    break;
  case URM12_MODULE_PID:
    Serial.print(distanceRaw * 1); Serial.println(" cm");
    break;
  case URM14_MODULE_PID:
    Serial.print(distanceRaw * 0.1); Serial.println(" mm");
    break;
  case URM15_MODULE_PID:
    Serial.print(distanceRaw * 0.1); Serial.println(" cm");
    break;
  default:
    Serial.println(distanceRaw);
    break;
  }

  /**
   * Obtain temperature measurement data, unit is celsius
   */
  float temperatureC = sensor.getTemperatureC();
  Serial.print("Temperature : ");  Serial.print(temperatureC);  Serial.println(" °C");

  if (URM14_MODULE_PID == sensor.basicInfo.PID) {
    Serial.println("-----------------read module power supply noise level--------------------");
    /**
     * Get the power supply noise level.
     * This parameter reflects the degree of influence of the power supply and environment on the sensor.
     *   The smaller the noise level, the more accurate the distance value obtained by the sensor.
     * uint8_t 0x00-0x0A corresponds to noise levels 0-10.
     */
    uint8_t noise = sensor.getNoiseLevel();
    Serial.print("Power supply noise level : ");  Serial.println(noise);
  }

  delay(1000);
}