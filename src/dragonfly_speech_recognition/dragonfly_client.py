import logging
import multiprocessing.connection
import os
import sys
import time

if os.name == 'nt':
    sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/grammar_parser/src/")

from grammar_parser.cfgparser import CFGParser

FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)


class DragonflyClient:

    def __init__(self, ip, port, timeout=5.0):
        self._address = (ip, port)
        self._timeout = timeout

    def _get_connection(self):
        start = time.time()
        while True:
            try:
                return multiprocessing.connection.Client(self._address)
            except OSError:
                if time.time() - start > self._timeout:
                    raise
                time.sleep(0.1)

    def recognize(self, grammar, target, is_preempt_requested=None):
        grammar_parser = CFGParser.fromstring(grammar)
        grammar_parser.verify(target)

        logging.info("Sending the recognition command")
        conn = self._get_connection()
        conn.send((grammar, target))
        while not conn.poll(.1):
            if is_preempt_requested and is_preempt_requested():
                conn.send(0)  # Send a cancel request
                return None

        # Now wait for result
        sentence = conn.recv()

        logging.info("Dragonfly Client received sentence %s", sentence)

        return grammar_parser.parse(target, sentence), sentence

    def restart_node(self):
        logging.info("Sending the restart command")

        conn = self._get_connection()
        conn.send('restart_node')
