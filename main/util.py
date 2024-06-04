import time
import queue
import multiprocessing


# ---------------------------------------------------------------------------- #
class DoubleQueue:
    def __init__(self):
        self.q1 = multiprocessing.Queue()
        self.q2 = multiprocessing.Queue()

    def put_q1(self, item):
        self.q1.put_nowait(item)
    def put_q2(self, item):
        self.q2.put_nowait(item)

    def last_q1(self, value):
        return last_from_queue(self.q1, value)
    def last_q2(self, value):
        return last_from_queue(self.q2, value)
    
    def close(self):
        self.q1.close()
        self.q2.close()

        # basically `allow_exit_without_flush`
        self.q1.cancel_join_thread()
        self.q2.cancel_join_thread()


class SingleQueue:
    def __init__(self):
        self.q = multiprocessing.Queue()

    def put(self, item):
        self.q.put_nowait(item)

    def last(self, value):
        return last_from_queue(self.q, value)

    def close(self):
        self.q.close()
        # basically `allow_exit_without_flush`
        self.q.cancel_join_thread()


def last_from_queue(q, last_value):
    value = last_value

    while True:
        try:
            value = q.get_nowait()
        except queue.Empty:
            break

    return value
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
def process_str(p):
    return f"Name: {p.name} PID: {p.pid} Exit code: {p.exitcode} Alive: {p.is_alive()}"


def create_process(target, args, name):
    process_name = f"{name}_process"
    p = multiprocessing.Process(target=target, args=args, name=process_name)
    p.daemon = True
    p.start()
    print(f"\nStarting {process_str(p)}...")
    return p

def close_process(p):
    print(f"\nClosing {p.name}...")

    print(f"1) {process_str(p)}")
    time.sleep(1)
    p.join(1)
    print(f"2) {process_str(p)}")

    for i in range(2):
        if p.is_alive():
            p.terminate()
            time.sleep(1)
            p.join(1)
            print(f"{3 + i}) {process_str(p)}")
        else:
            break

    try:
        # multiprocessing.Process.close() was added in Python 3.7
        # (catch does not exist for Python 3.6)
        p.close()
    except AttributeError:
        pass
# ---------------------------------------------------------------------------- #
