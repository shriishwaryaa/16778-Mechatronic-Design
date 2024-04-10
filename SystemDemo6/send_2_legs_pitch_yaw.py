import os
import time
import math
from timeit import default_timer as timer
import subprocess 
import time
import numpy as np
import threading
import serial

ser = serial.Serial('/dev/ttyACM1', 9600, timeout=1)

def init_serial():
  ser.reset_input_buffer()
  time.sleep(1)

def send_pitch(angle):
  # TODO : put while True if this doesnt work :)
  i = 0
  for i in range(0,4,2):
      ser.write((str(angle) + "$" + str(i) + '\n').encode('utf-8'))
      wait_for_ack()
      time.sleep(1)
      print("Finished sending:", i)

def send_yaw(angle):
  # TODO : put while True if this doesnt work :)
  for i in range(1,4,2):
      ser.write((str(angle) + "$" + str(i) + '\n').encode('utf-8'))
      wait_for_ack()
      time.sleep(1)
      print("Finished sending:", i) # Ensure Arduino knows the end of the data

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

  send_yaw(10)

  send_pitch(-5)

  ser.close()

if __name__=="__main__":
  main()
