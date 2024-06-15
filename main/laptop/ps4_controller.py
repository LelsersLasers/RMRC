import time
import requests
import threading
import laptop.consts
import pyPS4Controller.controller


INTERFACE = "/dev/input/js0"
MAX_JOYSTICK_VALUE = 32767


class PS4Controller(pyPS4Controller.controller.Controller):
    # ------------------------------------------------------------------------ #
    def __init__(self, video_capture_zero, **kwargs):
        pyPS4Controller.controller.Controller.__init__(self, **kwargs)
        
        self.left_y_value  = 0
        self.right_x_value = 0
        self.circle_down = False

        self.request_dict = {
            "left": 0,
            "right": 0,
            "last_time": time.time(),
            "outbound": False,
            "invert": False,
            "success": True,
        }
        self.request_threads = []

        self.base_url = laptop.consts.BASE_MOTOR_TEST_URL if video_capture_zero else laptop.consts.BASE_MOTOR_URL
    # ------------------------------------------------------------------------ #
    
    # ------------------------------------------------------------------------ #
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
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
    def calculate_power(self):
        y_input = self.left_y_value / MAX_JOYSTICK_VALUE
        x_input = self.right_x_value / MAX_JOYSTICK_VALUE

        if self.request_dict["invert"]: y_input = -y_input

        left_speed  = 0
        right_speed = 0

        if not self.circle_down:
            if y_input == 0:
                left_speed  =  x_input
                right_speed = -x_input
            else:
                left_speed  = y_input
                right_speed = y_input

                invert_mod = -1 if self.request_dict["invert"] else 1
                x_input *= invert_mod
                diagonal_multiplier = 1 - abs(x_input)

                if   x_input < 0: left_speed  *= diagonal_multiplier
                elif x_input > 0: right_speed *= diagonal_multiplier

        self.request_dict["left"]  = left_speed
        self.request_dict["right"] = right_speed
        self.request_dict["last_time"] = time.time()

        # -------------------------------------------------------------------- #
        t = threading.Thread(target=power_request_old, args=(self.request_dict, self.base_url))
        t.daemon = True
        t.start()
        # -------------------------------------------------------------------- #

        # -------------------------------------------------------------------- #
        # self.request_threads = [t for t in self.request_threads if t.is_alive()]
        # print(f"Threads: {len(self.request_threads)}")

        # if len(self.request_threads) == 0:
        #     # if no threads currently waiting to send something
        #     self.request_dict["outbound"] = False
        #     t = threading.Thread(target=power_request, args=(self.request_dict, self.base_url))
        #     t.daemon = True
        #     t.start()
        #     self.request_threads.append(t)
        # elif self.request_dict["outbound"]:
        #     # all existing threads are just waiting on the request
        #     # and no longer checking for new inputs
        #     self.request_dict["outbound"] = False
        #     t = threading.Thread(target=power_request, args=(self.request_dict, self.base_url))
        #     t.daemon = True
        #     t.start()
        #     self.request_threads.append(t)
        # else:
        #     # there is a thread that is currently available to repond to the new inputs
        #     print("Saved a get")
        # -------------------------------------------------------------------- #
    # ------------------------------------------------------------------------ #

    # ------------------------------------------------------------------------ #
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
    # def on_L3_up(self, value): pass
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
    # ------------------------------------------------------------------------ #
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def power_request(request_dict, base_url):
    orginal_time = request_dict["last_time"]
    time.sleep(laptop.consts.PS4_REQUEST_WAIT)
    new_time = request_dict["last_time"]

    if orginal_time != new_time:
        time.sleep(laptop.consts.PS4_REQUEST_WAIT)

    try:
        power_url = base_url + f"power/{request_dict['left']}/{request_dict['right']}"
        request_dict["outbound"] = True
        response = requests.get(power_url, timeout=laptop.consts.GET_TIMEOUT)
        data = response.json()
        request_dict["invert"] = data["invert"]
        request_dict["success"] = True
    except requests.exceptions.RequestException as e:
        print(f"{type(e)}: {power_url}")
        request_dict["success"] = False

def power_request_old(request_dict, base_url):
    try:
        power_url = base_url + f"power/{request_dict['left']}/{request_dict['right']}"
        response = requests.get(power_url, timeout=laptop.consts.GET_TIMEOUT)
        data = response.json()
        request_dict["invert"] = data["invert"]
        request_dict["success"] = True
    except requests.exceptions.RequestException as e:
        print(f"{type(e)}: {power_url}")
        request_dict["success"] = False
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
def process(video_capture_zero):
    while True:
        ps4 = PS4Controller(video_capture_zero=video_capture_zero, interface=INTERFACE, connecting_using_ds4drv=False)
        ps4.listen(timeout=-1)
# ---------------------------------------------------------------------------- #