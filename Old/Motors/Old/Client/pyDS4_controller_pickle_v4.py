from pyPS4Controller.controller import Controller
import pickle
import time

class MyController(Controller):
    def __init__(self, **kwargs):
        Controller.__init__(self, **kwargs)
        self.max = 150
        self.LeftX = 0
        #self.LeftY = 0  
        self.RightX = 0
        #self.RightY = 0
        self.Motor1 = 0
        self.Motor2 = 0  
        self.Motor3 = 0
        self.Motor4 = 0
        self.reset = 0

    def updateControl(self):
        # print(list([self.LeftX, self.LeftY, self.RightX, self.RightY]))
        if self.LeftX<16384 and self.LeftX>-16384:
            self.Motor3=round(self.LeftY/32768*self.max)
            self.Motor4=round(-self.LeftY/32768*self.max)
        if self.LeftX>=16384:
            self.Motor3=round(self.LeftX/32768*self.max)
            self.Motor4=round(-self.LeftX/32768*self.max)
        if self.LeftX<=-16384:
            self.Motor3=round(self.LeftX/32768*self.max)
            self.Motor4=round(-self.LeftX/32768*self.max)
        if self.RightX>=16384:
            self.Motor1=round(self.RightX/32768*self.max)
            self.Motor2=round(-self.RightX/32768*self.max)
        if self.RightX<=-16384:
            self.Motor1=round(self.RightX/32768*self.max)
            self.Motor2=round(-self.RightX/32768*self.max)
        if self.RightX<16384 and self.RightX>-16384:
            self.Motor1=round(self.RightX/32768*self.max)
            self.Motor2=round(-self.RightX/32768*self.max)
        StrNum=",".join([str(self.Motor1),str(self.Motor2),str(self.Motor3),str(self.Motor4),str(self.reset)])
        print("p:", StrNum)
        file=open('motorspeed.ps4','wb')
        pickle.dump(StrNum, file)
        file.close()
        # print(StrNum)
    

    def on_L3_up(self, value):
        print("L3 up ", value, end=" ")
        self.LeftX=value
        self.updateControl()
    def on_L3_down(self, value):
        print("L3 dn ", value, end=" ")
        self.LeftX=value
        self.updateControl()
    #def on_L3_left(self, value):
        #print("L3 lf ", end="")
        #self.LeftY=value
        #self.updateControl()
    #def on_L3_right(self, value):
        #print("L3 rt ", end="")
        #self.LeftY=value
        #self.updateControl()
    def on_L3_y_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        print("L3 y rest ", end="")
        self.LeftX=0
        self.updateControl()
    def on_L3_x_at_rest(self):
        """L3 joystick is at rest after the joystick was moved and let go off"""
        print("L3 x rest ", end="")
        self.LeftX=0
        self.updateControl()
    def on_R3_up(self, value):
        print("R3 up ", value, end=" ")
        self.RightX=value
        self.updateControl()
    def on_R3_down(self, value):
        print("R3 dn ", value, end=" ")
        self.RightX=value
        self.updateControl()
    #def on_R3_left(self, value):
        #print("R3 lf ", end="")
        #self.RightY=value
        #self.updateControl()
    #def on_R3_right(self, value):
        #print("R3 rt ", end="")
        #self.RightY=value
        #self.updateControl()
    def on_R3_y_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        print("R3 y rest ", end="")
        self.RightX=0
        self.updateControl()
    def on_R3_x_at_rest(self):
        """R3 joystick is at rest after the joystick was moved and let go off"""
        print("R3 x rest ", end="")
        self.RightX=0
        self.updateControl()
    def on_circle_press(self):
        print("Circle Press ", end="")
        self.reset=1
        self.updateControl()
    def on_circle_release(self):
        print("Circle Release ", end="")
        self.reset=0
        self.updateControl()

con = MyController(interface="/dev/input/js0", connecting_using_ds4drv=False)
con.listen()
