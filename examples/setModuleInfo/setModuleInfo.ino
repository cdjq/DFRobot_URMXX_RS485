/*!
 * @file  setModuleInfo.ino
 * @brief  Configure some basic parameters and obtain measurement data
 * @copyright  Copyright (c) 2024 DFRobot Co.Ltd (http://www.dfrobot.com)
 * @license  The MIT License (MIT)
 * @author   [qsjhyy](yihuan.huang@dfrobot.com)
 * @version  V1.0
 * @date  2024-11-01
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

 /**
  * DFRobot_URMXX constructor
  * Depending on the sensor used, open the corresponding macro
  * addr: modbus slave address(range1~247)or broadcast address(0x00)
  * If it's configured a broadcast address, send a broadcast packet, and all slaves on bus will process it but not respond
  */
DFRobot_URM08 sensor(/*addr =*/URMXX_DEFAULT_ADDR);
// DFRobot_URM12 sensor(/*addr =*/URMXX_DEFAULT_ADDR);
// DFRobot_URM14 sensor(/*addr =*/URMXX_DEFAULT_ADDR);
// DFRobot_URM15 sensor(/*addr =*/URMXX_DEFAULT_ADDR);


void setup(void)
{
  Serial.begin(115200);

  // After the sensor serial port configuration is modified and the sensor is restarted, 
  // reconfigure the following serial port initialization parameters based on the modified values
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
  Serial.println("Begin ok!\n");

  /***************** Sensor basic information configuration ******************************/

    /**
     * Set the module communication address, the default address is 0x000F
     * addr Device address to be set,(1~247 is 0x0001~0x00F7)
     */
  sensor.setADDR(0x000F);

  /**
   *  Set the module baud rate, the setting takes effect after power-off and restart, the default value is 9600
   *  addr The baud rate to be set：
   *  eBaudrate2400---2400 , eBaudrate4800---4800 , eBaudrate9600---9600 ,
   *  eBaudrate14400---14400 ,eBaudrate19200---19200 , eBaudrate38400---38400 ,
   *  eBaudrate57600---57600 , eBaudrate115200---115200 ,eBaudrate_1000000---1000000
   */
  sensor.setBaudrateMode(sensor.eBaudrate9600);

  /**
   * Set check bit and stop bit of the module
   * mode The mode to be set：
   * check bit：
   *       eCheckBitNone
   *       eCheckBitEven
   *       eCheckBitOdd
   * stop bit：
   *       eStopBit0p5
   *       eStopBit1
   *       eStopBit1p5
   *       eStopBit2
   */
  sensor.setCheckbitStopbit(sensor.eCheckBitNone | sensor.eStopBit2);

  if (URM08_MODULE_PID != sensor.basicInfo.PID) {
    /**
     * The user writes the temperature for the external compensation function
     */
    sensor.setExternalTemperatureC(25.0);
  }
  delay(1000);
}

void loop()
{
  if (URM08_MODULE_PID != sensor.basicInfo.PID) {
    Serial.println("\n-----------------set module--------------------");
    /**
     *  Configuration control register
     *  mode sURMXXConfig_t:
     *    configMode.tempCompSource : eInternalTemp: Use onboard temperature compensation,  eExternalTemp: Use external temperature compensation work
     *                          (requires user to write temperature data to external temperature compensation data register)
     *    configMode.tempCompMode : eTempCompEN: enable temperature compensation,  eTempCompDIS: disable temperature compensation
     *    configMode.autoMeasureMode : eAutoMeasureMode: auto measuring distance,  ePassiveMeasureMode: passive measuring distance
     *    configMode.trigPasvMeasure : In passive mode, write 1(eTrigPassiveMeasure) to the bit and the sensor will complete a distance measurement (about 300ms).
     *    configMode.reserved : reserved bit
     */
    DFRobot_URMXX::sURMXXConfig_t configMode = {
      .tempCompSource = sensor.eExternalTemp,
      .tempCompMode = sensor.eTempCompEN,
      .autoMeasureMode = sensor.ePassiveMeasureMode,
      .trigPasvMeasure = sensor.eTrigPassiveMeasure,
      .reserved = 0,
    };
    sensor.setControlRegister(configMode);
    delay(300);
  }

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
     * note: Only urm14 has this feature, so please actively unblock the following code when you want to view it.
     */
    // uint8_t noise = sensor.getNoiseLevel();
    // Serial.print("Temperature : ");  Serial.println(noise);
  }

  delay(1000);
}
