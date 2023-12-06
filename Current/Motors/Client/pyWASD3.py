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


    def press(self, stop_key, last_key, total_time, t0, one_press):
        if keyboard.is_pressed("w"):
            print("pressed: w")
            speed = -32767
            if (one_press == False):
                speed = -4000
            if keyboard.is_pressed("a"):
                print("+a")
                self.update('R3', speed)
                self.update('L3', speed/2)
            elif keyboard.is_pressed("d"):
                print("+d")
                self.update('R3', speed/2)
                self.update('L3', speed)

            elif keyboard.is_pressed("s"):
                print("+s")
                self.update('R3', 0)
                self.update('L3', 0)
            else:
                self.update('R3', speed)
                self.update('L3', speed) # w
            last_key = "w"
        elif keyboard.is_pressed("a"):
            print("pressed: a")
            speed = 32767
            if (one_press == False):
                speed = 4000
            if keyboard.is_pressed("w"):
                print("+w")
                self.update('R3', speed)
                self.update('L3', speed/2)
            elif keyboard.is_pressed("d"):
                print("+d")
                self.update('R3', 0)
                self.update('L3', 0)
            elif keyboard.is_pressed("s"):
                print("+s")
                self.update('R3', speed)
                self.update('L3', speed/2)
            else:
                self.update('R3', -speed)
                self.update('L3', speed) # a
            last_key = "a"
        elif keyboard.is_pressed("d"):
            print("pressed: d")
            speed = 32767
            if (one_press == False):
                speed = 4000
            if keyboard.is_pressed("w"):
                print("+w")
                self.update('R3', speed/2)
                self.update('L3', speed)
            elif keyboard.is_pressed("a"):
                print("+d")
                self.update('R3', 0)
                self.update('L3', 0)
            elif keyboard.is_pressed("s"):
                print("+s")
                self.update('R3', speed/2)
                self.update('L3', speed)
            else:
                self.update('R3', speed)
                self.update('L3', -speed) # d
            last_key = "d"
        elif keyboard.is_pressed("s"):
            speed = -32767
            if (one_press == False):
                speed = -4000
            print("pressed: s")
            if keyboard.is_pressed("w"):
                print("+w")
                self.update('R3', 0)
                self.update('L3', 0)
            elif keyboard.is_pressed("a"):
                print("+d")
                self.update('R3', speed)
                self.update('L3', speed/2)
            elif keyboard.is_pressed("d"):
                print("+s")
                self.update('R3', speed/2)
                self.update('L3', speed)
            else:
                self.update('R3', -speed)
                self.update('L3', -speed) # s
            last_key = "s"
        elif keyboard.is_pressed("p"):
            print("quit")
            stop_key = True
        elif keyboard.is_pressed("Shift"):
            print("pressed: Shift")
            if one_press == False:
                one_press = True
            else:
                one_press = False
        else:
            print("pressed: nothing")
            self.update('R3', 0)
            self.update('L3', 0)
        return stop_key, last_key, total_time, t0, one_press

stop_key = False
last_key = "q"
total_time = 0
t0 = time.time()
con = KeyControls()
one_press = True
while stop_key == False:
    stop_key, last_key, total_time, t0, one_press = con.press(stop_key,
    last_key, total_time, t0, one_press)
    keyboard.read_key()
quit()
