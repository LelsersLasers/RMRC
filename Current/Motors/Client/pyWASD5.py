import pickle
import keyboard
import time

CTRL_MAX = -32767


class KeyControls:
    values = {
        'R3':0,
        'L3':0,
        'CR':0,
        'M1':0,
        'M2':0,
        'M3':0,
        'M4':0,
        'M5':0,
        'M6':0,
    }
    def __init__(self):
        self.num = 1
    
    
    def resetMotorPositions(self):
        targets = [0, 0, 0, 0, 0, 0]
        while True:
            num_fin = 0
            for i in range(len(targets)):
                curr_key = 'M' + str(i + 1)
                diff = targets[i] - self.values[curr_key]
                if diff == 0:
                    num_fin += 1
                    continue 
                pos = diff > 0
                if pos == True:
                    self.values[curr_key] += 5
                else:
                    self.values[curr_key] -= 5
                self.update(curr_key, self.values[curr_key], True)
            time.sleep(0.2)
            if num_fin == len(targets):
                return
        
    def update(self, control, value, arm_on):
        self.values[control] = value
        motors = [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0]
        motors[0] = str(int(self.values['L3'] / CTRL_MAX * 330))
        motors[1] = str(int(self.values['R3'] / CTRL_MAX * 330))
        motors[2] = str(int(-self.values['L3'] / CTRL_MAX * 330))
        motors[3] = str(int(-self.values['R3'] / CTRL_MAX * 330))
        motors[4] = str(int(self.values['CR']))
        motors[5] = str(int(self.values['M1']))
        motors[6] = str(int(self.values['M2']))
        motors[7] = str(int(self.values['M3']))
        motors[8] = str(int(self.values['M4']))
        motors[9] = str(int(self.values['M5']))
        motors[10] = str(int(self.values['M6']))

        print(motors)
        with open('motorspeed.ps4', 'wb') as f:
            pickle.dump(motors, f)

    def press(self, stop_key, last_key, total_time, t0, one_press, arm_on):
        r = 0
        l = 0
        if arm_on == False:
            if keyboard.is_pressed("w"):
                speed = -32767
                if (one_press == False):
                    speed = -4000
                if keyboard.is_pressed("a"):
                    r = speed
                    l = 2*speed/5
                elif keyboard.is_pressed("d"):
                    r = 2*speed/5
                    l = speed
                elif keyboard.is_pressed("s"):
                    r = 0
                    l = 0
                else:
                    r = speed
                    l = speed # w
                last_key = "w"
                self.update('R3', r, False)
                self.update('L3', l, False)
            elif keyboard.is_pressed("a"):
                speed = 32767
                if (one_press == False):
                    speed = 4000
                if keyboard.is_pressed("w"):
                    r = speed
                    l = 2*speed/5
                elif keyboard.is_pressed("d"):
                    r = 0
                    l = 2*speed/5
                elif keyboard.is_pressed("s"):
                    r = speed
                    l = 2*speed/5
                else:
                    r = -speed
                    l = speed # a
                last_key = "a"
                self.update('R3', r, False)
                self.update('L3', l, False)
            elif keyboard.is_pressed("d"):
                speed = 32767
                if (one_press == False):
                    speed = 4000
                if keyboard.is_pressed("w"):
                    r = 2*speed/5
                    l = speed
                elif keyboard.is_pressed("a"):
                    r = 0
                    l = 0
                elif keyboard.is_pressed("s"):
                    r = 2*speed/5
                    l = speed
                else:
                    r = speed
                    l = -speed # d
                last_key = "d"
                self.update('R3', r, False)
                self.update('L3', l, False)
            elif keyboard.is_pressed("s"):
                speed = -32767
                if (one_press == False):
                    speed = -4000
                if keyboard.is_pressed("w"):
                    r = 0
                    l = 0
                elif keyboard.is_pressed("a"):
                    r = speed
                    l = 2*speed/5
                elif keyboard.is_pressed("d"):
                    r = 2*speed/5
                    l = speed
                else:
                    r = -speed
                    l = -speed # s
                last_key = "s"
                self.update('R3', r, False)
                self.update('L3', l, False)
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
                    
            elif keyboard.is_pressed("r"):
                m6_val = self.values['M6']
                m6_val -= 5
                if m6_val < 0:
                    m6_val = 0
                self.update('M6', m6_val, False)
            elif keyboard.is_pressed("e"):
                m6_val = self.values['M6']
                m6_val += 5
                if m6_val > 180:
                    m6_val = 180
                self.update('M6', m6_val, False)
                
            else:
                print("pressed: nothing")
                self.update('R3', 0, False)
                self.update('L3', 0, False)
        else:
            curr = 'M'+ str(self.num)
            curr_num = self.values[curr]
            if keyboard.is_pressed("w"):
                curr_num += 5
                motor_on = True
                if curr_num > 295:
                    curr_num = 295
            elif keyboard.is_pressed("s"):
                curr_num -= 5
                motor_on = True
                if curr_num < 0:
                    curr_num = 0
            elif keyboard.is_pressed("a"):
                self.num -= 1
                if self.num < 1:
                    self.num = 1
                curr = 'M' + str(self.num)
                curr_num = self.values[curr]
            elif keyboard.is_pressed("d"):
                self.num += 1
                if self.num > 5:
                    self.num = 5
                curr = 'M' + str(self.num)
                curr_num = self.values[curr]
            elif keyboard.is_pressed("x"):
                print("pressed: x")
                if self.values['CR'] == 0:
                    self.update('CR', 1, True)
                else:
                    self.update('CR', 0, True)
            self.values[curr] = curr_num   
            self.update(curr, curr_num, True)
            
            if keyboard.is_pressed("r"):
                m6_val = self.values['M6']
                m6_val -= 5
                if m6_val < 0:
                    m6_val = 0
                self.update('M6', m6_val, True)
            elif keyboard.is_pressed("e"):
                m6_val = self.values['M6']
                m6_val += 5
                if m6_val > 180:
                    m6_val = 180
                self.update('M6', m6_val, True)

        if keyboard.is_pressed("m"):
            print("pressed: m")
            if arm_on == False:
                arm_on = True
                print("Arm on.")
            else:
                arm_on = False
                print("Arm off.")
        if keyboard.is_pressed("p"):
            print("quit")
            self.resetMotorPositions()
            stop_key = True
            
        
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
