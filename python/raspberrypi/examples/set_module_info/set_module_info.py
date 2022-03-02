# -*- coding: utf-8 -*
'''!
  @file  set_module_info.py
  @brief  Configure the basic information and measurement parameters of the sensor
  @copyright  Copyright (c) 2010 DFRobot Co.Ltd (http://www.dfrobot.com)
  @license  The MIT License (MIT)
  @author  [qsjhyy](yihuan.huang@dfrobot.com)
  @version  V1.0
  @date  2021-07-23
  @url  https://github.com/DFRobot/DFRobot_RS01
'''
from __future__ import print_function
import sys
import os
sys.path.append(os.path.dirname(os.path.dirname(os.path.dirname(os.path.realpath(__file__)))))

from DFRobot_RS01 import *

sensor = DFRobot_RS01(addr = 0x000E)

def print_measure_data(number, distance, intensity):
  print("target: %d distance: %d: " %(number, distance), end='')
  for i in range(1, distance, 20):
    print("-", end='')
  print()

  print("target: %d intensity: %d: " %(number, intensity), end='')
  for i in range(1, intensity, 20):
    print("+", end='')
  print()


def setup():
  while (sensor.begin() == False):
    print ('Please check that the device is properly connected')
    time.sleep(3)
  print("sensor begin successfully!!!")

  '''
    Set the module communication address, the default address is 0x000E
    addr Device address to be set, (1~247 is 0x0001~0x00F7)
  '''
  sensor.set_ADDR(0x000E)

  '''
    Set the module baud rate, the setting takes effect after power-off and restart, the default is 115200
    mode Baud rate to be set: 
    E_BAUDRATE_2400---2400, E_BAUDRATE_4800---4800, E_BAUDRATE_9600---9600, 
    E_BAUDRATE_14400---14400, E_BAUDRATE_19200---19200, E_BAUDRATE_38400---38400, 
    E_BAUDRATE_57600---57600, E_BAUDRATE_115200---115200
  '''
  sensor.set_baudrate_mode(sensor.E_BAUDRATE_115200)

  '''
    Set check bit and stop bit of the module
    mode Mode to be set, perform OR operation on the following to get mode: 
         check bit:
              E_CHECKBIT_NONE
              E_CHECKBIT_EVEN
              E_CHECKBIT_ODD
         stop bit:
              E_STOPBIT_1
              E_STOPBIT_2
  '''
  sensor.set_checkbit_stopbit(sensor.E_CHECKBIT_NONE | sensor.E_STOPBIT_1)

  '''
    ---------------------------------------------
    starting_position: 
    Configure measurement start position value, the default value is 200(0x00C8),
    value Start position value, 70~6600(0x0046~0x19C8), can't be greater than stop position value
    ---------------------------------------------
    stop_position: 
    Configure measurement stop position value, the default value is 6000(0x1770),
    value Stop position value, 70~6600(0x0046~0x19C8), can't be less than start position set value
    ---------------------------------------------
    initial_threshold: 
    Configure the initial threshold, the default value is 400(0x0190),
    value Initial threshold, 100~10000(0x0064~0x2710)。
    ---------------------------------------------
    end_threshold: 
    Configure the end threshold, the default value is 400(0x0190),
    value End threshold, 100~10000(0x0064~0x2710)。
    ---------------------------------------------
    module_sensitivity: 
    Configure the module sensitivity, the default value is 0x0002,
    value Module sensitivity, 0x0000~0x0004。
    ---------------------------------------------
    comparison_offset: 
    Configure the comparison offset, the default value is 0(0x0000),
    value Comparison offset, -32768~32767(0x0000~0xFFFF)。
    ---------------------------------------------
  '''
  sensor.set_all_measurement_parameters(starting_position=500, stop_position=1500,
                                        initial_threshold=400, end_threshold=200,
                                        module_sensitivity=0x0002, comparison_offset=-100)

  '''
    Restore to factory setting
  '''
  #sensor.restore_factory_setting()


def loop():
  '''
    Read the module measured data
    Return the read data list
      The first element: the number of objects currently detected
      The second element: measured distance to the first object; the third element: measured intensity of the first object
      The fourth element: measured distance to the second object; the fifth element: measured intensity of the second object
      The sixth element: measured distance to the third object; the seventh element: measured intensity of the third object
      The eighth element: measured distance to the fourth object; the ninth element: measured intensity of the fourth object
      The tenth element: measured distance to the fifth object; the eleventh element: measured intensity of the fifth object
  '''
  buf_data = sensor.read_measurement_data()
  if 0 != len(buf_data):
    ''' the number of objects currently detected '''
    print("target amount:  %d" %buf_data[0])

    ''' measured data '''
    for i in range(1, 6):
      print_measure_data(i, buf_data[i*2-1], buf_data[i*2])
  else:
    print("Failed to read measurement data!!!")
  print()

  time.sleep(1)


if __name__ == "__main__":
  setup()
  while True:
    loop()
