from pyPS4Controller.controller import Controller
import threading 

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
    
    def handle_x(self):
      print("Inside handle x function")
      pass

    def handle_tri(self):
      print("Inside handle tri function")
      pass

    def handle_cir(self):
      print("Inside handle circle function ")
      pass

    def handle_sqr(self):
      print("Inside the handle square function ")
      pass

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

  controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
  # you can start listening before controller is paired, as long as you pair it within the timeout window
  controller.listen(timeout=60)

  main_joystick_thread = threading.Thread(target=controller)
  main_joystick_thread.start()
  main_joystick_thread.join()
