import requests
import laptop.consts
import pyPS4Controller.controller


INTERFACE = "/dev/input/js0"
MAX_JOYSTICK_VALUE = 32768


class PS4Controller(pyPS4Controller.controller.Controller):
    def __init__(self, video_capture_zero, **kwargs):
        pyPS4Controller.controller.Controller.__init__(self, **kwargs)
        
        self.left_y_value  = 0
        self.right_x_value = 0
        self.invert = False

        self.base_url = laptop.consts.BASE_PRIMARY_TEST_URL if video_capture_zero else laptop.consts.BASE_PRIMARY_URL
    
    def on_L3_up(self, value):
        print("on_L3_up", value)
        self.left_y_value = value
        self.calculate_power()
    def on_L3_down(self, value):
        print("on_L3_down", value)
        self.left_y_value = value
        self.calculate_power()
    def on_L3_y_at_rest(self):
        print("on_L3_y_at_rest")
        self.left_y_value = 0
        self.calculate_power()

    def on_R3_left(self, value):
        print("on_R3_left", value)
        self.right_x_value = value
        self.calculate_power()
    def on_R3_right(self, value):
        print("on_R3_right", value)
        self.right_x_value = value
        self.calculate_power()
    def on_R3_x_at_rest(self):
        print("on_R3_x_at_rest")
        self.calculate_power()
        self.right_x_value = 0

    def calculate_power(self):
        y_input = self.left_y_value / MAX_JOYSTICK_VALUE
        x_input = self.right_x_value / MAX_JOYSTICK_VALUE

        if self.invert: y_input = -y_input

        left_speed  = 0
        right_speed = 0

        if y_input == 0:
            left_speed  =  x_input
            right_speed = -x_input
        else:
            left_speed  = y_input
            right_speed = y_input

            invert_mod = -1 if self.invert else 1
            x_input *= invert_mod
            diagonal_multiplier = 1 - abs(x_input)

            if   x_input < 0: speed_left  *= diagonal_multiplier
            elif x_input > 0: speed_right *= diagonal_multiplier

        try:
            power_url = self.base_url + f"power/{left_speed}/{right_speed}"
            response = requests.get(power_url, timeout=0.5)
            data = response.json()
            self.invert = data["invert"]
        except requests.exceptions.RequestException as e:
            print(f"{type(e)}: {power_url}")


def thread(video_capture_zero):
    # TODO: try/except/while for waiting for it to be plugged in
    ps4 = PS4Controller(video_capture_zero=video_capture_zero, interface=INTERFACE, connecting_using_ds4drv=False)
    ps4.listen()