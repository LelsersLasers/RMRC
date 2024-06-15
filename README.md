# RMRC

Code written by The Knight Sky for Bishop's RMRC team

## The Team

- Dr. J
- Dr. Keller
- Audrey Lin
- Ryan Zhu
- Mia Gover
- Sienna Li
- Millan Kumar
- Victor Joulin-Batejat
- Charlie Fredberg
- Grace Yao
- Amy Yan

## Setup (TODO update!)

The code is designed to just be cloned and run.
The main changes are:
- Changing `LATENCY_TIMER` in the library's code (Jetson and laptop)
    - `grep -ir "LATENCY_TIMER" /`
    - `LATENCY_TIMER = 4`
- Changing `listen()` for the ps4 controller (laptop only)
    - `sudo find / -name "controller.py"`
    - Replace `wait_for_interface()` and `read_events()` with:
```
def wait_for_interface():
    print("Waiting for interface: {} to become available . . .".format(self.interface))
    i = 0
    while i < timeout or timeout == -1:
        if os.path.exists(self.interface):
            print("Successfully bound to: {}.".format(self.interface))
            on_connect_callback()
            return
        i += 1
        time.sleep(1)
    print("Timeout({} sec). Interface not available.".format(timeout))

def read_events():
    try:
        return _file.read(self.event_size)
    except IOError:
        print("Interface lost. Device disconnected?")
        on_disconnect_callback()
```
- Changing the `easyocr` order of checking GPU backends (Jetson only)