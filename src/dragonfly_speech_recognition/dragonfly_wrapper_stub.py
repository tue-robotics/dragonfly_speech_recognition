import thread
import time
import random
from grammar_parser.cfgparser import CFGParser


class DragonflyWrapper:
    def __init__(self):
        self._grammar = None
        self._root = None
        self._recognition_result = None

    def set_grammar(self, grammar, root):
        self._grammar = grammar
        self._root = root
        thread.start_new_thread(self._recognition_thread, ())

    def unset_grammar(self):
        self._grammar = None

    def _recognition_thread(self):
        time.sleep(random.random() * 5)
        if self._grammar:
            try:
                cfg_parser = CFGParser.fromstring(self._grammar)
                self._recognition_result = cfg_parser.random(self._root)
            except Exception as e:
                print e
                self._recognition_result = " "

    def get_recognition(self):
        r = self._recognition_result if self._recognition_result else None
        if r:
            self._recognition_result = None
            self._grammar = None
            self._root = None
        return r
