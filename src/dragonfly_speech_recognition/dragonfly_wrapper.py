import logging

import pythoncom
import winsound
import sys
import os
import time
from Queue import Queue
from get_dragonfly_grammar import get_dragonfly_grammar
from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine


FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)

# make dragonfly less verbose
logging.getLogger('engine.compiler').setLevel(logging.INFO)
logging.getLogger('grammar.decode').setLevel(logging.INFO)
logging.getLogger('grammar.begin').setLevel(logging.INFO)
logging.getLogger('compound.parse').setLevel(logging.INFO)


class DragonflyWrapper:
    def __init__(self):
        self._engine = Sapi5InProcEngine()
        self._engine.connect()
        self._dragonfly_grammar = None
        self._result_queue = Queue()

    def set_grammar(self, grammar, target):
        self._dragonfly_grammar, self._result_queue = get_dragonfly_grammar(grammar, target)

        # Now load the grammar
        start = time.time()
        self._dragonfly_grammar.load()
        logger.info("Loading the grammar took %.3f seconds", time.time() - start)

    def unset_grammar(self):
        if self._dragonfly_grammar:
            self._dragonfly_grammar.unload()

    def get_recognition(self):
        # Pump the win32 com iface
        pythoncom.PumpWaitingMessages()
        if self._result_queue.empty():
            return None
        return self._result_queue.get_nowait()
