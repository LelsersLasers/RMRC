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
    print(f"\nStarting {name} thread...")
    process_name = f"{name}_process"
    thread = multiprocessing.Process(target=target, args=args, name=process_name)
    thread.daemon = True
    thread.start()
    print(f"{name} process pid: {thread.pid}")
    return thread

def close_thread(t):
    t.join(1)
    t.terminate()
    t.join(1)

    try:    
        t.close()
    except:
        pass
# ---------------------------------------------------------------------------- #