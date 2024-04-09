import os
import time
import math
from timeit import default_timer as timer
import subprocess 
import time
import numpy as np
import threading
import serial
import matplotlib.pyplot as plt

from pyPS4Controller.controller import Controller

ctrl = [1, 0, 0.5, 0.25, 0.25, 0.5, 
1, 0.5, 0.5, 0.25, 0.75, 0.5, 
1, 0, 0.5, 0.25, 0.25, 0.5, 
1, 0, 0.5, 0.25, 0.75, 0.5, 
1, 0.5, 0.5, 0.25, 0.25, 0.5, 
1, 0, 0.5, 0.25, 0.75, 0.5]

angles = np.zeros(100)

def radians_to_degrees(angle_radians):
  return math.radians_to_degrees(angle_radians)

def control_signal(amplitude, phase, duty_cycle, array_dim=100):
  # Forward motion
  assert(amplitude >= 0 and amplitude <= 1)
  assert(phase >= 0 and phase <= 1)
  assert(duty_cycle >= 0 and duty_cycle <= 1)
  command = np.zeros(array_dim)
  # create a 'top-hat function'
  up_time = array_dim * duty_cycle
  temp = [amplitude if i < up_time else -amplitude for i in range(0, array_dim)]
  # smoothing kernel
  kernel_size = int(array_dim / 10)
  kernel = np.zeros(int(2 * kernel_size + 1))
  # Standard deviation of the bell curve 
  sigma = kernel_size / 3
  # This loop calculates the value of each point for the Gaussian kernel 
  for i in range(0, len(kernel)):
      kernel[i] =  math.exp(-(i - kernel_size) * (i - kernel_size) / (2 * sigma**2)) / (sigma * math.sqrt(math.pi))

  # Sum of all the elements in the kernel which is of size 21
  sum = np.sum(kernel)
  # smooth the function
  for i in range(0, array_dim):
    command[i] = 0
    for d in range(1, kernel_size + 1):
        if i - d < 0:
            command[i] += temp[array_dim + i - d] * kernel[kernel_size - d]
        else:
            command[i] += temp[i - d] * kernel[kernel_size - d]
    command[i] += temp[i] * kernel[kernel_size]
    for d in range(1, kernel_size + 1):
        if i + d >= array_dim:
            command[i] += temp[i + d - array_dim] * kernel[kernel_size + d]
        else:
            command[i] += temp[i + d] * kernel[kernel_size + d]
    command[i] /= sum
  # shift according to the phase
  # here we are shifting the signal according to the phase
  # just multiply the provided phase by the size of the array
  start = int(math.floor(array_dim * phase))
  current = 0
  for i in range(start, array_dim):
      angles[current] = command[i]
      current += 1
  for i in range(0, start):
      angles[current] = command[i]
      current += 1
  assert(len(angles) == array_dim)

  for i in range(len(angles)):
    curr_angle = angles[i]
    angles[i] = math.degrees(curr_angle)

  # print(angles)

  # return final_command

class joystick_controller(Controller):

  def __init__(self, **kwargs):
      Controller.__init__(self, **kwargs)
  
  def joystick_listen(self):
      self.listen()
      
  def on_x_press(self):
    print("moving_bkwd")

  def on_triangle_press(self):
    print("moving_fwd - Pose before")
    print("Done moving forward - Pose after")

  def on_x_release(self):
      pass

  def on_triangle_release(self):
      pass

  def on_square_release(self):
      pass

  def on_circle_release(self):
      pass

def main():
  print ("Inside main function")

  control_signal(1, 0, 0.5, 100)

  ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)
  ser.reset_input_buffer()

  time.sleep(1)

  dummy = 20

  while True:
    # ser.write((str(dummy) + '\n').encode('utf-8'))  # Ensure Arduino knows the end of the data
    for angle in angles:
      ser.write((str(angle) + '\n').encode('utf-8'))  # Ensure Arduino knows the end of the data
      time.sleep(10)
      print("Done sending", angle)

  ser.close()

if __name__ == "__main__":
  main()
