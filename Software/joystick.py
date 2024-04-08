from pyPS4Controller.controller import Controller

class JoystickController(Controller):
  def __init__(self, **kwargs):
      Controller.__init__(self, **kwargs)
  
  def joystick_listen(self):
      self.listen()
      
  def on_x_press(self):
      print("moving_bkwd")