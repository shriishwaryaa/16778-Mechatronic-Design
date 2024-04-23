# Main program which runs on the Raspberry Pi
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import threading 
import serial
from collections import defaultdict

# our libraries
# from .joystick import JoystickController

# Global list to store all the threads 
dancing_threads = []

class DanceRobot():
  def __init__(self):
    print("Hello from Dance Robot")
    # self.Joystick = JoystickController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    self.Estop = False
    # We have 6 legs and 3 Arduinos
    # Each Arduino will power 2 legs - which means 4 motors 
    self.serial_ports = ["/dev/arduino_0", "/dev/arduino_imu", "/dev/arduino_2"]

    # self.imu_serial_dev = "/dev/arduino_imu"

    self.serial_port_handlers = []
    self.tripod_1 = [0,3,4]
    # self.tripod_2 = [0]

    legs = [0, 3, 4]
    self.legs_to_motors = defaultdict(list)

    for leg in legs:
      # motors = [(2 * leg), (2 * leg) + 1]
      # Motor 0 is yaw motor
      # Motor 1 is pitch motor
      
      self.legs_to_motors[leg].append(2*leg)
      self.legs_to_motors[leg].append((2*leg) + 1)

    self.forward_angles_basic = [5, -5, -5, -5]
    self.backward_angles_basic = [5, -5, -5]
    self.map_leg_serial = defaultdict(list)

    # self.imu_serial_port = serial.Serial(self.imu_serial_dev, 115200, timeout=1)
    # self.imu_serial_port.reset_input_buffer()

  def init_serial(self):
    for i in range(len(self.serial_ports)):
      serial_port = serial.Serial(self.serial_ports[i], 9600, timeout=1)
      serial_port.reset_input_buffer()
      self.serial_port_handlers.append(serial_port)
      self.map_leg_serial[2*i].append(serial_port)
      self.map_leg_serial[(2*i)+1].append(serial_port)

    time.sleep(5) 

  
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

  def wait_for_ack(self, ser):
    while True:
      try:
        # line = ser.readline().decode('utf-8').rstrip()
        line=ser.readline().decode('utf-8').rstrip()
      # print("This is the line:" + line)
        while line != "ok":
          print("This is the line:" + line)
          line=ser.readline().decode('utf-8').rstrip()
        return
      except UnicodeDecodeError:
        print("Received bytes couldn't be decoded as UTF-8. Ensure the Arduino is sending valid UTF-8 strings.")

  def write_angle_pitch(self, leg, serial_port, angle):
    motor_id = self.legs_to_motors[leg][1] % 2
    print("Sending pitch to motor ", motor_id, "leg ", leg, serial_port.port)
    cmd = str(angle) + "$" + str(motor_id)
    print("Cmd: ", cmd)
    serial_port.write((cmd + '\n').encode('utf-8'))
    time.sleep(self.calculate_delay(angle)) 
    # self.wait_for_ack(serial_port)

  def write_angle_yaw(self, leg, serial_port, angle):
    motor_id = self.legs_to_motors[leg][0] % 2
    print("Sending pitch to motor ", motor_id, "leg ", leg, serial_port.port)
    cmd = str(angle) + "$" + str(motor_id)
    print("Cmd: ", cmd)
    serial_port.write((cmd + '\n').encode('utf-8'))
    time.sleep(self.calculate_delay(angle)) 
    # self.wait_for_ack(serial_port)

  def send_pitch(self, legs, angle):
    print("Sending pitch angle to legs ", legs)
    pitch_threads = []

    for leg in legs:
      serial_port = self.map_leg_serial[leg][0]
      thread = threading.Thread(target=self.write_angle_pitch, args=[leg, serial_port, angle])
      thread.start()
      pitch_threads.append(thread)

    for t in pitch_threads:
      t.join()
      pitch_threads.remove(t)
    
    print("Pitch done")
      
  def send_yaw(self, legs, angle):
    print("Sending yaw angle to legs ", legs)
    yaw_threads = []

    for leg in legs:
      serial_port = self.map_leg_serial[leg][0]
      thread = threading.Thread(target=self.write_angle_yaw, args=[leg, serial_port, angle])
      thread.start()
      yaw_threads.append(thread)

    for t in yaw_threads:
      t.join()
      yaw_threads.remove(t)

    print("Yaw done")
  def move_forward_basic(self):
    # Basic tripod - legs 0, 3, 4 lift up move forward and down
    #                legs 1, 2, 5 lift up and move forward and then down
    # Pitch angles - [5, -5]
    # Yaw angles - [10]
    for i in range(3):
      angle = 5
      # send pitch angle to the tripod one
      self.send_pitch(self.tripod_1, angle)
      time.sleep(1)

      angle = 5
      self.send_yaw(self.tripod_1, angle)
      time.sleep(1)

      angle = -5
      # send pitch angle to the tripod one
      self.send_pitch(self.tripod_1, angle)
      time.sleep(1)

      angle = -5
      # send pitch angle to the tripod one
      self.send_yaw(self.tripod_1, angle)
      time.sleep(1)

    # angle = 10
    # self.send_yaw(self.tripod_2, angle)
    # time.sleep(3)

    # angle = -5
    # # send pitch angle to the tripod one
    # self.send_pitch(self.tripod_2, angle)
    # time.sleep(3)
    
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

    angle = 5
    # send pitch angle to the tripod one
    self.send_pitch(self.tripod_2, angle)

    angle = -10
    self.send_yaw(self.tripod_2, angle)

    angle = -5
    # send pitch angle to the tripod one
    self.send_pitch(self.tripod_2, angle)

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

  # imu_thread = threading.Thread(target=TeamA.read_imu_data)
  # imu_thread.start()
  # dancing_threads.append(imu_thread)

  print("Moving forward now")
  TeamA.move_forward_basic()

  # print("Moving backward now")
  # TeamA.move_backward_basic()

  for t in dancing_threads:
    t.join()

if __name__=='__main__':
  main()
