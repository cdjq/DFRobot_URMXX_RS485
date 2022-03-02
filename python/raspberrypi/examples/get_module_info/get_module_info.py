# -*- coding: utf-8 -*
'''!
  @file get_module_info.py
  @brief Read all the register data from the sensor
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
  print("target: %d distance: %d:" %(number, distance), end='')
  for i in range(1, distance, 20):
    print("-", end='')
  print()

  print("target: %d intensity: %d:" %(number, intensity), end='')
  for i in range(1, intensity, 20):
    print("+", end='')
  print()


def setup():
  while (sensor.begin() == False):
    print ('Please check that the device is properly connected')
    time.sleep(3)
  print("sensor begin successfully!!!")


def loop():
  print()
  '''
    Read the module basic information
    Return the read data list
      The first element: module PID
      The second element: module VID
      The third element: module communication address
      The fourth element: module baud rate
      The fifth element: module check bit and stop bit
      The sixth element: firmware version number
  '''
  buf = sensor.read_basic_info()
  if 0 != len(buf):
    # Module PID, the default value is 0x01E9 (The highest two bits are used to judge SKU type 00:SEN¡¢01:DFR¡¢10:TEL, The next 14 bits are used as num)(SEN0489)
    print("PID: 0x%x"%buf[0])

    # Module VID, the default value is 0x3343 (represent manufacturer DFRobot)
    print("VID: 0x%x"%buf[1])

    # Module communication address, the default value is 0x000E, module device address (1~247)
    print("mailing address: 0x%x"%buf[2])

    # Module baud rate, the default value is 0x0009:
    # 0x0001---2400  0x0002---4800  0x0003---9600  0x0004---14400  0x0005---19200
    # 0x0006---38400  0x0007---57600  0x0008---115200  0x0009---1000000
    print("baudrate: 0x%x"%buf[3])

    # Module check bit and stop bit, the default value is 0x0001
    # Check bit: 1 represents none; 2 represents even; 3 represents odd
    # Stop bit: 1bit; 2bit
    print("check bit: %d"%(buf[4]>>8))
    print("stop bit: %d"%((buf[4]+1)/2))

    # Firmware version number: 0x1000 represents V1.0.0.0
    print("versions: 0x%x "%buf[5])
  else:
    print("Failed to read basic information")
  print()

  '''
    Read the module measurement parameters currently configured
    Return the read data list
      The first element: current measurement start position set value
      The second element: current measurement stop position set value
      The third element: current initial threshold set value
      The fourth element: current end threshold set value
      The fifth element: current module sensitivity set value
      The sixth element: current comparison offset set value
  '''
  buf = sensor.read_measurement_config()
  if 0 != len(buf):
    # Current measurement start position set value, 0x0046~0x19C8, can't be greater than stop position set value
    print("starting position: %d"%buf[0])

    # Current measurement stop position set value, 0x0046~0x19C8, can't be less than start position set value
    print("stop position: %d"%buf[1])

    # Current initial threshold set value
    print("initial threshold: %d"%buf[2])

    # Current end threshold set value
    print("end threshold: %d"%buf[3])

    # Current module sensitivity set value
    print("module sensitivity: %d"%buf[4])

    # Current comparison offset set value
    print("comparison offset: 0x%x "%buf[5])
  else:
    print("Failed to read basic information")
  print()

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
    print("target amount: %d" %buf_data[0])

    ''' measured data '''
    for i in range(1, 6):
      print_measure_data(i, buf_data[i*2-1], buf_data[i*2])
  else:
    print("Failed to read measurement data!!!")
  print()

  time.sleep(3)

if __name__ == "__main__":
  setup()
  while True:
    loop()
