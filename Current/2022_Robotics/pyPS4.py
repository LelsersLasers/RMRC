from pyPS4Controller.controller import Controller
import pickle

CTRL_MAX = -32767

class MyController(Controller):
    values = {
        'R3': 0,
        'L3': 0,
        'CR': 0,
    }

    def update(self, control, value):
        self.values[control] = value
        motors = [0, 0, 0, 0, 0]
        # Left Front
        motors[0] = str(int(self.values['L3'] / CTRL_MAX * 330))
        # Right Front
        motors[1] = str(int(self.values['R3'] / CTRL_MAX * 330))
        # Left Rear
        motors[2] = str(int(-self.values['L3'] / CTRL_MAX * 330))
        # Right Rear
        motors[3] = str(int(-self.values['R3'] / CTRL_MAX * 330))
        # Button for flipper
        motors[4] = str(self.values['CR'])
        command_packet = motors[0] + ',' + motors[1] + ',' + motors[2] + ',' + motors[3] + ',' + motors[4]
        print(command_packet)
        with open('motorspeed.ps4', 'wb') as f:
            pickle.dump(command_packet, f)

    def on_R3_up(self, value):
        self.update("R3", value)

    def on_R3_down(self, value):
        self.update("R3", value)

    def on_R3_y_at_rest(self):
        self.update("R3", 0)
        
    def on_L3_up(self, value):
        self.update("L3", value)

    def on_L3_down(self, value):
        self.update("L3", value)
        
    def on_L3_y_at_rest(self):
        self.update("L3", 0)

    def on_circle_press(self): 
        self.update("CR", 1)

    def on_circle_release(self):
        self.update("CR", 0)
    
    def on_L1_press(self): pass
    def on_L1_release(self): pass
    def on_R1_press(self): pass
    def on_R1_release(self): pass
    def on_L2_press(self, value): pass
    def on_L2_release(self): pass
    def on_R2_press(self,value): pass
    def on_R2_release(self): pass
    def on_L3_left(self, value): pass
    def on_L3_right(self, value): pass
    def on_L3_x_at_rest(self): pass
    def on_R3_left(self, value): pass
    def on_R3_right(self, value): pass
    def on_R3_x_at_rest(self): pass
    def on_up_arrow_press(self): pass
    def on_down_arrow_press(self): pass
    def on_up_down_arrow_release(self): pass
    def on_left_arrow_press(self): pass
    def on_right_arrow_press(self): pass
    def on_left_right_arrow_release(self): pass
    def on_x_press(self): pass
    def on_x_release(self): pass
    def on_triangle_press(self): pass
    def on_triangle_release(self): pass
    def on_square_press(self): pass
    def on_square_release(self): pass
    def on_playstation_button_press(self): pass
    def on_playstation_button_release(self): pass
    def on_options_press(self): pass
    def on_options_release(self): pass
    def on_share_press(self): pass
    def on_share_release(self): pass


con = MyController(interface='/dev/input/js0', connecting_using_ds4drv=False)
con.listen()
