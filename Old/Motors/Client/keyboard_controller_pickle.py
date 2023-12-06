import pickle
import curses

def main():
    win = curses.initscr()
    curses.noecho()
    reset = 0
    while True:
        ch = win.getch()
        if chr(ch) == "w":
            motor1 = 250
            motor2 = 250
            motor3 = 250
            motor4 = 250
        if chr(ch) == "s":
            motor1 = -150
            motor2 = -150
            motor3 = -150
            motor4 = -150
        if chr(ch) == "a":
            motor1 = 100
            motor2 = -100
            motor3 = 100
            motor4 = -100
        if chr(ch) == "d":
            motor1 = 100
            motor2 = -100
            motor3 = 100
            motor4 = -100           
        strNum=",".join([str(motor1),str(motor2),str(motor3),str(motor4),str(reset)])
        print("p:", strNum)
        with open('motorspeed.ps4','wb') as f:
            pickle.dump(strNum, f)
        # print(strNum)
    
if __name__ == "__main__":
    main()
