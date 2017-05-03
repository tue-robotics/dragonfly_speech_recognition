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
logging.getLogger().setLevel(logging.INFO)
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
        pythoncom.PumpWaitingMessages()  # Get rid of all recognition results from previous cycle

        self._dragonfly_grammar = get_dragonfly_grammar(grammar, target, self._result_queue)

        # Now load the grammar
        start = time.time()
        self._dragonfly_grammar.load()
        logger.info("Loading the grammar took %.3f seconds", time.time() - start)

        sound_path = os.path.dirname(os.path.realpath(__file__)) + "/../../data/grammar_loaded.wav"
        logger.info("PlaySound %s", sound_path)
        winsound.PlaySound(sound_path, winsound.SND_ASYNC)

    def unset_grammar(self):
        if self._dragonfly_grammar:
            self._dragonfly_grammar.unload()
            pythoncom.PumpWaitingMessages()  # Get rid of all leftover messages in the queue

    def get_recognition(self):
        logger.debug("Main thread recognition Q [id=%s, qsize=%d]", id(self._result_queue), self._result_queue.qsize())
        # Pump the win32 com iface
        pythoncom.PumpWaitingMessages()
        if self._result_queue.empty():
            return None
        return self._result_queue.get_nowait()
