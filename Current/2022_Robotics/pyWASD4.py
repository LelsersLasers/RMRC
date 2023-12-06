import pickle
import keyboard
import time
from serial_test import getNumServos, updateServos

CTRL_MAX = -32767


class KeyControls:
    values = {
        'R3':0,
        'L3':0,
        'CR':0,
    }
    def useArm(self, motors):
        incoming = updateServos()
        if len(incoming) == 2*getNumServos():
            for i in range(1, len(incoming), 2):
                 motors.append(str(incoming[i]))
        print(motors)
        with open('motorspeed.ps4', 'wb') as f:
            pickle.dump(motors, f)
    def update(self, control, value, arm_on):
        self.values[control] = value
        motors = [0, 0, 0, 0, 0]
        if arm_on == False:
            motors[0] = str(int(self.values['L3'] / CTRL_MAX * 330))
            motors[1] = str(int(self.values['R3'] / CTRL_MAX * 330))
            motors[2] = str(int(-self.values['L3'] / CTRL_MAX * 330))
            motors[3] = str(int(-self.values['R3'] / CTRL_MAX * 330))
            motors[4] = str(self.values['CR'])
        else:
            while arm_on == True:
                self.useArm(motors)
                if keyboard.is_pressed("m"):
                    print("pressed: m")
                    arm_on = False
                    print("Arm stopped.")
                motors = [0, 0, 0, 0, 0]
        print(motors)
        with open('motorspeed.ps4', 'wb') as f:
            pickle.dump(motors, f)

    def press(self, stop_key, last_key, total_time, t0, one_press, arm_on):
        if arm_on == False:
            if keyboard.is_pressed("w"):
                print("pressed: w")
                speed = -32767
                if (one_press == False):
                    speed = -4000
                if keyboard.is_pressed("a"):
                    print("+a")
                    self.update('R3', speed, False)
                    self.update('L3', 2*speed/5, False)
                elif keyboard.is_pressed("d"):
                    print("+d")
                    self.update('R3', 2*speed/5, False)
                    self.update('L3', speed, False)

                elif keyboard.is_pressed("s"):
                    print("+s")
                    self.update('R3', 0, False)
                    self.update('L3', 0, False)
                else:
                    self.update('R3', speed, False)
                    self.update('L3', speed, False) # w
                last_key = "w"
            elif keyboard.is_pressed("a"):
                print("pressed: a")
                speed = 32767
                if (one_press == False):
                    speed = 4000
                if keyboard.is_pressed("w"):
                    print("+w")
                    self.update('R3', speed, False)
                    self.update('L3', 2*speed/5, False)
                elif keyboard.is_pressed("d"):
                    print("+d")
                    self.update('R3', 0, False)
                    self.update('L3', 0, False)
                elif keyboard.is_pressed("s"):
                    print("+s")
                    self.update('R3', speed, False)
                    self.update('L3', 2*speed/5, False)
                else:
                    self.update('R3', -speed, False)
                    self.update('L3', speed, False) # a
                last_key = "a"
            elif keyboard.is_pressed("d"):
                print("pressed: d")
                speed = 32767
                if (one_press == False):
                    speed = 4000
                if keyboard.is_pressed("w"):
                    print("+w")
                    self.update('R3', 2*speed/5, False)
                    self.update('L3', speed, False)
                elif keyboard.is_pressed("a"):
                    print("+d")
                    self.update('R3', 0, False)
                    self.update('L3', 0, False)
                elif keyboard.is_pressed("s"):
                    print("+s")
                    self.update('R3', 2*speed/5, False)
                    self.update('L3', speed, False)
                else:
                    self.update('R3', speed, False)
                    self.update('L3', -speed, False) # d
                last_key = "d"
            elif keyboard.is_pressed("s"):
                speed = -32767
                if (one_press == False):
                    speed = -4000
                print("pressed: s")
                if keyboard.is_pressed("w"):
                    print("+w")
                    self.update('R3', 0, False)
                    self.update('L3', 0, False)
                elif keyboard.is_pressed("a"):
                    print("+d")
                    self.update('R3', speed, False)
                    self.update('L3', 2*speed/5, False)
                elif keyboard.is_pressed("d"):
                    print("+s")
                    self.update('R3', 2*speed/5, False)
                    self.update('L3', speed, False)
                else:
                    self.update('R3', -speed, False)
                    self.update('L3', -speed, False) # s
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
            elif keyboard.is_pressed("x"):
                print("pressed: x")
                if self.values['CR'] == 0:
                    self.update('CR', 1, False)
                else:
                    self.update('CR', 0, False)
            else:
                print("pressed: nothing")
                self.update('R3', 0, False)
                self.update('L3', 0, False)
        else:
            self.update('R3', 0, True)
            self.update('L3', 0, True)
        if keyboard.is_pressed("m"):
            print("pressed: m")
            if arm_on == False:
                arm_on = True
            else:
                arm_on = False
        return stop_key, last_key, total_time, t0, one_press, arm_on

stop_key = False
last_key = "q"
total_time = 0
t0 = time.time()
con = KeyControls()
one_press = True
arm_on = False
while stop_key == False:
    stop_key, last_key, total_time, t0, one_press, arm_on = con.press(stop_key,
    last_key, total_time, t0, one_press, arm_on)
    keyboard.read_key()
quit()
