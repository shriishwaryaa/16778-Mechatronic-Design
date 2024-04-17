from pyPS4Controller.controller import Controller
import threading 
import serial
import time

from collections import defaultdict

serial_ports = ["/dev/arduino_0", "/dev/arduino_1"]
serial_port_handlers = []

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
    
    def handle_x(self):
      # Move motor 1 by 5 degrees
      print("Inside handle x function")
      motor_id = 0
      angle = 5

      cmd = str(angle) + "$" + str(motor_id)
      print("Command", cmd)
      serial_ports[0].write((cmd + '\n').encode('utf-8'))

    def handle_tri(self):
      print("Inside handle tri function")
      motor_id = 1
      angle = 5

      cmd = str(angle) + "$" + str(motor_id)
      print("Command", cmd)
      serial_ports[0].write((cmd + '\n').encode('utf-8'))

    def handle_cir(self):
      print("Inside handle circle function ")
      motor_id = 0
      angle = -5

      cmd = str(angle) + "$" + str(motor_id)
      print("Command", cmd)
      serial_ports[0].write((cmd + '\n').encode('utf-8'))

    def handle_sqr(self):
      print("Inside the handle square function ")
      motor_id = 0
      angle = -5

      cmd = str(angle) + "$" + str(motor_id)
      print("Command", cmd)
      serial_ports[0].write((cmd + '\n').encode('utf-8'))

    def on_x_press(self):
      print("Moving backward")
      # Spawn a thread to move backward
      thread = threading.Thread(target=self.handle_x)
      thread.start()
      thread.join()

    def on_triangle_press(self):
      print("Moving forward")
      thread = threading.Thread(target=self.handle_tri)
      thread.start()
      thread.join()

    def on_square_press(self):
      print("Turning")
      thread = threading.Thread(target=self.handle_sqr)
      thread.start()
      thread.join()

    def on_circle_press(self):
      print("Stopping")
      thread = threading.Thread(target=self.handle_cir)
      thread.start()
      thread.join()

def main():
  print("Inside main joystick function")

  global serial_ports
  global serial_port_handlers

  for i in range(len(serial_ports)):
    serial_port = serial.Serial(serial_ports[i], 9600, timeout=1)
    serial_port.reset_input_buffer()
    serial_port_handlers.append(serial_port)

  time.sleep(5)

  print("Serial ports initialized")

  controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
  # you can start listening before controller is paired, as long as you pair it within the timeout window
  controller.listen(timeout=60)

  main_joystick_thread = threading.Thread(target=controller)
  main_joystick_thread.start()
  main_joystick_thread.join()

  for s in serial_port_handlers:
    s.close()

