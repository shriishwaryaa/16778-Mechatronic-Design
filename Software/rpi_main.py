# Main program which runs on the Raspberry Pi
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import threading 

# our libraries
from .joystick import JoystickController

# Global list to store all the threads 
dancing_threads = []

class DanceRobot():
  def __init__(self):
    print("Hello from Dance Robot")
    self.Joystick = JoystickController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    self.Estop = False
    self.serial_ports = ["/dev/arduino_0", "/dev/arduino_1"]
    self.serial_port_handlers = []

    self.tripod_1 = [0, 2, 4]
    self.tripod_2 = [1, 3, 5]
    self.forward_angles_basic = [5, 10, -5]
    self.backward_angles_basic = [5, -10, -5]

  def init_serial(self):
    for i in range(len(self.serial_ports)):
      serial_port = serial.Serial(self.serial_ports[i], 9600, timeout=1)
      serial_port.reset_input_buffer()
      self.serial_port_handlers.append(serial_port)
  
  def calculate_delay(self, angle):
  # 5 degrees to 2 seconds
  # angle is in degrees
    delay = (abs(angle) // 10) * 2
    return delay
  
  def move_forward_basic():
    # Basic tripod - legs 0, 2, 4 lift up move forward and down
    #                legs 1, 3, 5 lift up and move forward and then down
    for i in range(len(self.forward_angles_basic)):
      self.serial_port_handlers[].write((str(angle) + "$" + str(self.forward_angles_basic[i]) + '\n').encode('utf-8'))
      time.sleep(calculate_delay(angle))


def main():
  # Initialize Dance Robot
  TeamA = DanceRobot()
  # Initialize all the serial ports
  TeamA.init_serial()

  # Spawn a thread to listen to Joystick 
  joystick_thread = threading.Thread(target=TeamA.Joystick.joystick_listen)
  joystick_thread.start()
  dancing_threads.append(joystick_thread)

  for t in dancing_threads:
    t.join()

if __name__=='__main__':
  main()
