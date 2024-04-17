from pyPS4Controller.controller import Controller
import threading 

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def handle_x(self):
      print("Inside handle x function")
      for i in range(1000000):
        print("x: ", i)

    def handle_tri(self):
      print("Inside handle tri function")
      for i in range(1000000):
        print("tri: ", i)

    def handle_cir(self):
      print("Inside handle circle function ")
      pass

    def handle_sqr(self):
      print("Inside the handle square function ")
      pass

    def on_x_press(self):
      print("Moving backward")
      thread_x = threading.Thread(target=self.handle_x)
      thread_x.start()
      thread_x.join()

    def on_triangle_press(self):
      print("Moving forward")
      thread_t = threading.Thread(target=self.handle_tri)
      thread_t.start()
      thread_t.join()

def main():
  print("Inside main joystick function")

  controller = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
  # you can start listening before controller is paired, as long as you pair it within the timeout window
  controller.listen(timeout=60)

  main_joystick_thread = threading.Thread(target=controller)
  main_joystick_thread.start()
  main_joystick_thread.join()

if __name__=="__main__":
  main()
