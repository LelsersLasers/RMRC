import requests
import asyncio
import laptop.consts
import pyPS4Controller.controller


INTERFACE = "/dev/input/js0"
MAX_JOYSTICK_VALUE = 32767


class PS4Controller(pyPS4Controller.controller.Controller):
    def __init__(self, video_capture_zero, **kwargs):
        pyPS4Controller.controller.Controller.__init__(self, **kwargs)
        
        self.left_y_value  = 0
        self.right_x_value = 0
        self.invert = False
        self.circle_down = False

        self.base_url = laptop.consts.BASE_PRIMARY_TEST_URL if video_capture_zero else laptop.consts.BASE_PRIMARY_URL
    
    def on_L3_up(self, value):
        self.left_y_value = value
        self.calculate_power()
    def on_L3_down(self, value):
        self.left_y_value = value
        self.calculate_power()
    def on_L3_y_at_rest(self):
        self.left_y_value = 0
        self.calculate_power()

    def on_R3_left(self, value):
        self.right_x_value = value
        self.calculate_power()
    def on_R3_right(self, value):
        self.right_x_value = value
        self.calculate_power()
    def on_R3_x_at_rest(self):
        self.calculate_power()
        self.right_x_value = 0

    def on_circle_press(self):
        self.circle_down = True
        self.left_y_value  = 0
        self.right_x_value = 0
        self.calculate_power()
    def on_circle_release(self):
        self.circle_down = False
        self.left_y_value  = 0
        self.right_x_value = 0
        self.calculate_power()

    def calculate_power(self):
        y_input = self.left_y_value / MAX_JOYSTICK_VALUE
        x_input = self.right_x_value / MAX_JOYSTICK_VALUE

        if self.invert: y_input = -y_input

        left_speed  = 0
        right_speed = 0

        if not self.circle_down:
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

        print(left_speed, right_speed)

        self.invert = asyncio.run(power_request(self.base_url, left_speed, right_speed))

        # try:
        #     power_url = self.base_url + f"power/{left_speed}/{right_speed}"
        #     response = requests.get(power_url, timeout=0.5)
        #     data = response.json()
        #     self.invert = data["invert"]
        # except requests.exceptions.RequestException as e:
        #     # print(f"{type(e)}: {power_url}")
        #     ...

    # Overriding defaults so avoid prints
    def on_x_press(self): pass
    def on_x_release(self): pass
    def on_triangle_press(self): pass
    def on_triangle_release(self): pass
    # def on_circle_press(self): pass
    # def on_circle_release(self): pass
    def on_square_press(self): pass
    def on_square_release(self): pass
    def on_L1_press(self): pass
    def on_L1_release(self): pass
    def on_L2_press(self, value): pass
    def on_L2_release(self): pass
    def on_R1_press(self): pass
    def on_R1_release(self): pass
    def on_R2_press(self, value): pass
    def on_R2_release(self): pass
    def on_up_arrow_press(self): pass
    def on_up_down_arrow_release(self): pass
    def on_down_arrow_press(self): pass
    def on_left_arrow_press(self): pass
    def on_left_right_arrow_release(self): pass
    def on_right_arrow_press(self): pass
    def on_L3_up(self, value): pass
    # def on_L3_down(self, value): pass
    def on_L3_left(self, value): pass
    def on_L3_right(self, value): pass
    # def on_L3_y_at_rest(self): pass
    def on_L3_x_at_rest(self): pass
    def on_L3_press(self): pass
    def on_L3_release(self): pass
    def on_R3_up(self, value): pass
    def on_R3_down(self, value): pass
    # def on_R3_left(self, value): pass
    # def on_R3_right(self, value): pass
    def on_R3_y_at_rest(self): pass
    # def on_R3_x_at_rest(self): pass
    def on_R3_press(self): pass
    def on_R3_release(self): pass
    def on_options_press(self): pass
    def on_options_release(self): pass
    def on_share_press(self): pass
    def on_share_release(self): pass
    def on_playstation_button_press(self): pass
    def on_playstation_button_release(self): pass

async def power_request(base_url, left_speed, right_speed):
    try:
        power_url = base_url + f"power/{left_speed}/{right_speed}"
        response = requests.get(power_url, timeout=0.5)
        data = response.json()
        invert = data["invert"]
        return invert
    except requests.exceptions.RequestException as e:
        # print(f"{type(e)}: {power_url}")
        ...


def thread(video_capture_zero):
    while True:
        ps4 = PS4Controller(video_capture_zero=video_capture_zero, interface=INTERFACE, connecting_using_ds4drv=False)
        ps4.listen()