import pickle
import keyboard
import time

CTRL_MAX = -32767


class KeyControls:
    values = {
        'R3':0,
        'L3':0,
    }

    def update(self, control, value):
        self.values[control] = value
        motors = [0, 0, 0, 0]
        motors[0] = str(int(self.values['L3'] / CTRL_MAX * 330))
        motors[1] = str(int(self.values['R3'] / CTRL_MAX * 330))
        motors[2] = str(int(-self.values['L3'] / CTRL_MAX * 330))
        motors[3] = str(int(-self.values['R3'] / CTRL_MAX * 330))
        command_packet = motors[0] + ',' + motors[1] + ',' + motors[2] + ',' + motors[3] + ',0'
        print(command_packet)
        with open('motorspeed.ps4', 'wb') as f:
            pickle.dump(command_packet, f)


    def press(self, stop_key):
        if keyboard.is_pressed("w"):
            print("pressed: w")
            if keyboard.is_pressed("a"):
                print("+a")
                self.update('R3', 0)
                self.update('L3', 32767)

            elif keyboard.is_pressed("d"):
                print("+d")
                self.update('R3', 32767)
                self.update('L3', 0)

            elif keyboard.is_pressed("s"):
                print("+s")
                self.update('R3', 0)
                self.update('L3', 0)
            else:
                self.update('R3', 32767)
                self.update('L3', 32767) # w
        elif keyboard.is_pressed("a"):
            print("pressed: a")
            if keyboard.is_pressed("w"):
                print("+w")
                self.update('R3', 0)
                self.update('L3', 32767)
            elif keyboard.is_pressed("d"):
                print("+d")
                self.update('R3', 0)
                self.update('L3', 0)

            elif keyboard.is_pressed("s"):
                print("+s")
                self.update('R3', -32767)
                self.update('L3', 0)
            else:
                self.update('R3', -32767)
                self.update('L3', 32767) # a
        elif keyboard.is_pressed("d"):
            print("pressed: d")
            if keyboard.is_pressed("w"):
                print("+w")
                self.update('R3', 32767)
                self.update('L3', 0)
            elif keyboard.is_pressed("a"):
                print("+d")
                self.update('R3', 0)
                self.update('L3', 0)

            elif keyboard.is_pressed("s"):
                print("+s")
                self.update('R3', 0)
                self.update('L3', -32767)
            else:
                self.update('R3', 32767)
                self.update('L3', -32767) # d
        elif keyboard.is_pressed("s"):
            print("pressed: s")
            if keyboard.is_pressed("w"):
                print("+w")
                self.update('R3', 0)
                self.update('L3', 0)
            elif keyboard.is_pressed("a"):
                print("+d")
                self.update('R3', -32767)
                self.update('L3', 0)

            elif keyboard.is_pressed("d"):
                print("+s")
                self.update('R3', 0)
                self.update('L3', -32767)
            else:
                self.update('R3', -32767)
                self.update('L3', -32767) # s
        elif keyboard.is_pressed("p"):
            print("quit")
            stop_key = True
        else:
            print("pressed: nothing")
            self.update('R3', 0)
            self.update('L3', 0)
        return stop_key

con = KeyControls()
stop_key = False
while stop_key == False:
    stop_key = con.press(stop_key)
    time.sleep(0.05)
quit()
