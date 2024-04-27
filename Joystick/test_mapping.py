from pyPS4Controller.controller import Controller

class TestController(Controller):
    def on_any_button_press(self, button_id):
        print(f"Button {button_id} pressed")

    def on_any_button_release(self, button_id):
        print(f"Button {button_id} released")

def main():
    print("Testing controller inputs...")
    controller = TestController(interface="/dev/input/js0")
    controller.listen()

if __name__ == '__main__':
    main()
