from pyPS4Controller.controller import Controller

class JoystickController(Controller):
  def __init__(self, interface, **kwargs):
    self.interface=interface
    Controller.__init__(self, **kwargs)
      
  def on_x_press(self):
    print("X pressed")
  
  def on_triangle_press(self):
    print("Triangle pressed")

  def on_square_press(self):
    print("Triangle pressed")

  def on_circle_press(self):
    print("Triangle pressed") 