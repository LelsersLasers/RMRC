import time
import signal

# ---------------------------------------------------------------------------- #
class GracefulKiller:
    # "captures" SIGINT and SIGTERM signals to allow for graceful exit
    def __init__(self):
        # SIGINT =  Ctrl+C
        signal.signal(signal.SIGINT, self.exit_gracefully)

        # TODO: should have this or not?
        # SIGTERM =  kill
        # signal.signal(signal.SIGTERM, self.exit_gracefully)

        self.kill_now = False
    
    def exit_gracefully(self, *args):
        self.kill_now = True
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class FPSController:
    def __init__(self, start_delta = 1/30):
        self.t0 = time.time()
        self.t1 = time.time()
        self.delta = start_delta

    def update(self):
        self.t1 = time.time()
        self.delta = self.t1 - self.t0
        self.t0 = self.t1

        return self.delta
    
    def fps(self):
        if self.delta == 0:
            return -1
        else:
            return 1 / self.delta
# ---------------------------------------------------------------------------- #


# ---------------------------------------------------------------------------- #
class DoubleState:
    def __init__(self, s1, s2):
        self.s1 = s1
        self.s2 = s2

    def update_s1(self, dq):
        self.s1 = dq.last_q1(self.s1)
    def update_s2(self, dq):
        self.s2 = dq.last_q2(self.s2)

    def put_s1(self, dq):
        dq.put_q1(self.s1)
    def put_s2(self, dq):
        dq.put_q2(self.s2)
# ---------------------------------------------------------------------------- #