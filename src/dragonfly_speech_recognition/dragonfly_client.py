import multiprocessing.connection
import logging
import os
import sys

if os.name == 'nt':
    sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/grammar_parser/src/")

from grammar_parser.cfgparser import CFGParser


FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)


class DragonflyClient:

    def __init__(self, ip, port):
        self._address = (ip, port)

    def recognize(self, grammar, target, is_preempt_requested=None):
        grammar_parser = CFGParser.fromstring(grammar)
        grammar_parser.verify(target)

        conn = multiprocessing.connection.Client(self._address)
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

        conn = multiprocessing.connection.Client(self._address)
        conn.send('restart_node')

