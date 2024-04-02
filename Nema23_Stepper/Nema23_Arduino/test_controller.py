from pyPS4Controller.controller import Controller

class MyController(Controller):

    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)

    def on_x_press(self):
        print("Hello World")

    def on_x_release(self):
        print("Goodbye World")

controller = MyController(interface = "/dev/input/js0",
                          connecting_using_ds4drv=False)

controller.listen(timeout=60)
