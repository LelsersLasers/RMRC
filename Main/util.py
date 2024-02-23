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
def create_thread(target, args, name):
    process_name = f"{name}_process"
    thread = multiprocessing.Process(target=target, args=args, name=process_name)
    thread.daemon = True
    thread.start()
    print(f"\nStarting {thread.name=}: {thread.pid=} {thread.is_alive()=}")
    return thread

def close_thread(t):
    # wait 2.5 seconds for thread to finish otherwise terminate
    
    # TODO: does joint actually call terminate?

    print(f"\nClosing {t.name=} thread...")

    print(f"1) {t.name=} {t.pid=} {t.exitcode=} {t.is_alive()=}")
    t.join(2.5)
    print(f"2) {t.name=} {t.pid=} {t.exitcode=} {t.is_alive()=}")

    try:
        # multiprocessing.Process.terminate() was added in Python 3.7
        # (catch does not exist for Python 3.6)
        t.close()
    except:
        pass
# ---------------------------------------------------------------------------- #