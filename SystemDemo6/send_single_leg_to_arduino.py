import os
import time
import math
from timeit import default_timer as timer
import subprocess 
import time
import numpy as np
import threading
import serial

ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

def init_serial():
  ser.reset_input_buffer()
  time.sleep(1)

def send_pitch(angle):
  # TODO : put while True if this doesnt work :)
  i = 0
  ser.write((str(angle) + "$" + str("0") + '\n').encode('utf-8'))  # Ensure Arduino knows the end of the data
  print("Finished sending pitch")

def send_yaw(angle):
  # TODO : put while True if this doesnt work :)
  ser.write((str(angle) + "$" + str("1") + '\n').encode('utf-8'))  # Ensure Arduino knows the end of the data
  print("Finished sending yaw")

def wait_for_ack():
  while True:
    try:
      # line = ser.readline().decode('utf-8').rstrip()
      line=ser.readline().decode('utf-8').rstrip()
     # print("This is the line:" + line)
      while line != "ok":
        # print("This is the line:" + line)
         line=ser.readline().decode('utf-8').rstrip()
      return
    except UnicodeDecodeError:
      print("Received bytes couldn't be decoded as UTF-8. Ensure the Arduino is sending valid UTF-8 strings.")

def main():
  init_serial()
  print("Initialized serial")

  # send pitch angle to both motors first
  send_pitch(5)
  wait_for_ack()
  time.sleep(1)

  send_yaw(10)
  wait_for_ack()
  time.sleep(1)

  send_pitch(0)
  wait_for_ack()
  time.sleep(1)

  ser.close()

if __name__=="__main__":
  main()
~                                                                                                                                                                                                           
                                                                                                                                                                                          6,8           All
