import multiprocessing.connection


class DragonflyClient:

    def __init__(self, ip, port):
        self._address = (ip, port)

    def recognize(self, grammar, root, is_preempt_requested=None):
        conn = multiprocessing.connection.Client(self._address)
        conn.send((grammar, root))
        while not conn.poll(.1):
            if is_preempt_requested and is_preempt_requested():
                return None
        return conn.recv()
