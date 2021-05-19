import os
import sys
from threading import Thread
import traceback

if os.name == 'nt':
    sys.path.append(os.path.dirname(os.path.realpath(__file__)) + "/grammar_parser/src/")
from grammar_parser.cfgparser import CFGParser


class DragonflyWrapper:
    def __init__(self):
        self._grammar = None
        self._target = None
        self._sentence = ""
        self._reset()

    def _reset(self):
        self._grammar = None
        self._target = None
        self._sentence = None

    def set_grammar(self, grammar, target):
        self._grammar = grammar
        self._target = target
        Thread(target=self._recognition_thread).start()

    def unset_grammar(self):
        self._reset()

    def _recognition_thread(self):
        if self._grammar:
            try:
                cfg_parser = CFGParser.fromstring(self._grammar)
                self._sentence = cfg_parser.get_random_sentence(self._target)
            except Exception as e:
                print(f"Could not get random string: {e}\n{traceback.format_exc()}")
                self._sentence = ""
        self._grammar = None

    def get_recognition(self):
        if self._grammar is None:
            s = self._sentence
            self._reset()
            return s

        return None
