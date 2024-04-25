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

'''
on_x_press
on_x_release
on_triangle_press
on_triangle_release
on_square_press
on_square_release
on_circle_press
on_circle_release
on_down_arrow_press
on_up_down_arrow_release
on_left_arrow_press
on_left_right_arrow_release
on_right_arrow_press
on_left_right_arrow_release
on_up_arrow_press
on_up_down_arrow_release
on_L1_press
on_L1_release
on_L2_press: 32767
on_L2_press: 7431
on_L2_release
on_R1_press
on_R1_release
on_R2_press: -338
on_R2_press: 32767
on_R2_press: 11485
on_R2_release
'''