# Main program which runs on the Raspberry Pi
import os
import time
import numpy as np
import matplotlib.pyplot as plt
import threading 

# our libraries
from .joystick import JoystickController

dancing_threads = []

class DanceRobot():
  def __init__(self):
    print("Hello from Dance Robot")
    self.Joystick = JoystickController(interface="/dev/input/js0", connecting_using_ds4drv=False)
    self.Estop = False

def main():
  # Initialize Dance Robot
  TeamA = DanceRobot()

  # Spawn a thread to listen to Joystick 
  joystick_thread = threading.Thread(target=TeamA.Joystick.joystick_listen)
  joystick_thread.start()
  dancing_threads.append(joystick_thread)

  for t in dancing_threads:
    t.join()

if __name__=='__main__':
  main()