# Main program which runs on the Raspberry Pi
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import threading 
import serial
from collections import defaultdict

# our libraries
from .joystick import JoystickController

# Global list to store all the threads 
dancing_threads = []

class DanceRobot():
  def __init__(self):
    print("Hello from Dance Robot")
    self.Joystick = JoystickController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    self.Estop = False
    self.serial_ports = ["/dev/arduino_0", "/dev/arduino_1", "/dev/arduino_2"]
    self.serial_port_handlers = []
    self.tripod_1 = [0,3,4]
    self.tripod_2 = [1,2,5]

    legs = [0, 1, 2, 3, 4, 5]
    self.legs_to_motors = defaultdict(list)

    for leg in legs:
      # motors = [(2 * leg), (2 * leg) + 1]
      self.legs_to_motors[leg].append(2*leg)
      self.legs_to_motors[leg].append((2*leg) + 1)

    self.forward_angles_basic = [5, 10, -5]
    self.backward_angles_basic = [5, -10, -5]
    self.map_leg_serial = defaultdict(list)

  def init_serial(self):
    for i in range(len(self.serial_ports)):
      serial_port = serial.Serial(self.serial_ports[i], 9600, timeout=1)
      serial_port.reset_input_buffer()
      self.serial_port_handlers.append(serial_port)
      self.map_leg_serial[2*i].append(serial_port)
      self.map_leg_serial[(2*i)+1].append(serial_port) 

  
  def calculate_delay(self, angle):
  # 5 degrees to 2 seconds
  # angle is in degrees
    delay = (abs(angle) // 10) * 2
    return delay
  
  def send_pitch(self, leg, angle):
    # TODO - verify that the odd motors are the pitch motors
    motor_id = self.legs_to_motors[leg][1]
    serial_port = self.map_leg_serial[leg] 
    serial_port.write((str(angle) + "$" + str(self.legs_to_motors[leg][0]) + '\n').encode('utf-8'))
    time.sleep(self.calculate_delay(angle))

  def send_yaw(self, leg, angle):
    # TODO - verify that the odd motors are the yaw motors
    motor_id = self.legs_to_motors[leg][0]
    serial_port = self.map_leg_serial[leg] 
    serial_port.write((str(angle) + "$" + str(self.legs_to_motors[leg][0]) + '\n').encode('utf-8'))
    time.sleep(self.calculate_delay(angle))

  def move_forward_basic(self):
    # Basic tripod - legs 0, 3, 4 lift up move forward and down
    #                legs 1, 2, 5 lift up and move forward and then down
    # Pitch angles - [5, -5]
    # Yaw angles - [10]
    fwd_threads = []
    angle = 5

    for leg in self.tripod_1:
      new_thread = threading.Thread(target=self.send_pitch, args=[leg, angle])
      new_thread.start()
      fwd_threads.append(new_thread)

    angle = 10

    for leg in self.tripod_1:
      new_thread = threading.Thread(target=self.send_yaw, args=[leg, angle])
      new_thread.start()
      fwd_threads.append(new_thread)

    angle = -5
    for leg in self.tripod_1:
      new_thread = threading.Thread(target=self.send_pitch, args=[leg, angle])
      new_thread.start()
      fwd_threads.append(new_thread)

    for t in fwd_threads:
      t.join()
    
    print("Done moving forward")
  
  def move_backward_basic(self):
    # Basic tripod - legs 0, 3, 4 lift up move forward and down
    #                legs 1, 2, 5 lift up and move forward and then down
    # Pitch angles - [5, -5]
    # Yaw angles - [10]
    bkwd_threads = []
    angle = 5

    for leg in self.tripod_1:
      new_thread = threading.Thread(target=self.send_pitch, args=[leg, angle])
      new_thread.start()
      bkwd_threads.append(new_thread)

    angle = -10

    for leg in self.tripod_1:
      new_thread = threading.Thread(target=self.send_yaw, args=[leg, angle])
      new_thread.start()
      bkwd_threads.append(new_thread)

    angle = -5
    for leg in self.tripod_1:
      new_thread = threading.Thread(target=self.send_pitch, args=[leg, angle])
      new_thread.start()
      bkwd_threads.append(new_thread)

    for t in bkwd_threads:
      t.join()
    
    print("Done moving backward")

def main():
  # Initialize Dance Robot
  TeamA = DanceRobot()
  # Initialize all the serial ports
  TeamA.init_serial()

  # Spawn a thread to listen to Joystick 
  # joystick_thread = threading.Thread(target=TeamA.Joystick.joystick_listen)
  # joystick_thread.start()
  # dancing_threads.append(joystick_thread)

  print("Moving forward now")
  TeamA.move_forward_basic()

  print("Moving backward now")
  TeamA.move_backward_basic()

  for t in dancing_threads:
    t.join()

if __name__=='__main__':
  main()
