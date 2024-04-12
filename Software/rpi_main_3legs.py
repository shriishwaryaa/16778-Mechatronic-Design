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
    # We have 6 legs and 3 Arduinos
    # Each Arduino will power 2 legs - which means 4 motors 
    self.serial_ports = ["/dev/arduino_0", "/dev/arduino_1", "/dev/arduino_2"]

    self.imu_serial_dev = "/dev/arduino_imu"

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

    self.imu_serial_port = serial.Serial(self.imu_serial_dev, 115200, timeout=1)
    self.imu_serial_port.reset_input_buffer()

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
  
  def read_imu_data(self):
    # this needs to read pitch, roll and yaw
    while True:
      line = self.imu_serial_port.readline().decode('utf-8').strip()
      if line:
          print("IMU reading")
          print(line)
      time.sleep(0.1)

  def write_angle_pitch(self, leg, serial_port, angle):
    serial_port.write((str(angle) + "$" + str(self.legs_to_motors[leg][0]) + '\n').encode('utf-8'))
    time.sleep(self.calculate_delay(angle)) 

  def write_angle_yaw(self, leg, serial_port, angle):
    serial_port.write((str(angle) + "$" + str(self.legs_to_motors[leg][1]) + '\n').encode('utf-8'))
    time.sleep(self.calculate_delay(angle)) 

  def send_pitch(self, legs, angle):
    print("Sending pitch angle to legs ", legs)
    pitch_threads = []

    for leg in legs:
      serial_port = self.map_leg_serial[leg]
      thread = threading.Thread(target=self.write_angle_pitch, args=[leg, serial_port, angle])
      thread.start()
      pitch_threads.append(thread)

    for t in pitch_threads:
      t.join()
      
  def send_yaw(self, legs, angle):
    print("Sending yaw angle to legs ", legs)
    yaw_threads = []

    for leg in legs:
      serial_port = self.map_leg_serial[leg]
      thread = threading.Thread(target=self.write_angle_yaw, args=[leg, serial_port, angle])
      thread.start()
      yaw_threads.append(thread)

    for t in yaw_threads:
      t.join()

  def move_forward_basic(self):
    # Basic tripod - legs 0, 3, 4 lift up move forward and down
    #                legs 1, 2, 5 lift up and move forward and then down
    # Pitch angles - [5, -5]
    # Yaw angles - [10]
    angle = 5
    # send pitch angle to the tripod one
    self.send_pitch(self.tripod_1, angle)

    angle = 10
    self.send_yaw(self.tripod_1, angle)

    angle = -5
    # send pitch angle to the tripod one
    self.send_pitch(self.tripod_1, angle)
    
    print("Done moving forward")
  
  def move_backward_basic(self):
    # Basic tripod - legs 0, 3, 4 lift up move forward and down
    #                legs 1, 2, 5 lift up and move forward and then down
    # Pitch angles - [5, -5]
    # Yaw angles - [10]
    angle = 5
    # send pitch angle to the tripod one
    self.send_pitch(self.tripod_1, angle)

    angle = -10
    self.send_yaw(self.tripod_1, angle)

    angle = -5
    # send pitch angle to the tripod one
    self.send_pitch(self.tripod_1, angle)
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

  imu_thread = threading.Thread(target=TeamA.read_imu_data)
  imu_thread.start()
  dancing_threads.append(imu_thread)

  print("Moving forward now")
  TeamA.move_forward_basic()

  print("Moving backward now")
  TeamA.move_backward_basic()

  for t in dancing_threads:
    t.join()

if __name__=='__main__':
  main()
