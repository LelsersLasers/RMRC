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


import multiprocessing
def last_from_queue(q, last_value):
    value = last_value
    test = False

    while True:
        try:
            value = q.get_nowait()
            test = True
        except queue.Empty:
            break

    if not test:
        print(multiprocessing.current_process().name, "\tlast_from_queue: No new value")
    return value
# ---------------------------------------------------------------------------- #

# ---------------------------------------------------------------------------- #
def thread_str(t):
    return f"Name: {t.name} PID: {t.pid} Exit code: {t.exitcode} Alive: {t.is_alive()}"


def create_thread(target, args, name):
    process_name = f"{name}_process"
    thread = multiprocessing.Process(target=target, args=args, name=process_name)
    thread.daemon = True
    thread.start()
    print(f"\nStarting {thread_str(thread)}...")
    return thread

def close_thread(t):
    print(f"\nClosing {t.name} thread...")

    print(f"1) {thread_str(t)}")
    time.sleep(1)
    t.join(1)
    print(f"2) {thread_str(t)}")

    for i in range(2):
        if t.is_alive():
            t.terminate()
            time.sleep(1)
            t.join(1)
            print(f"{3 + i}) {thread_str(t)}")
        else:
            break

    try:
        # multiprocessing.Process.close() was added in Python 3.7
        # (catch does not exist for Python 3.6)
        t.close()
    except AttributeError:
        pass
# ---------------------------------------------------------------------------- #
