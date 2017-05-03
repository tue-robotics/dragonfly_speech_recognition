import thread
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
        thread.start_new_thread(self._recognition_thread, ())

    def unset_grammar(self):
        self._reset()

    def _recognition_thread(self):
        if self._grammar:
            try:
                cfg_parser = CFGParser.fromstring(self._grammar)
                self._sentence = cfg_parser.get_random_sentence(self._target)
            except Exception as e:
                print e
                self._sentence = ""
        self._grammar = None

    def get_recognition(self):
        if self._grammar is None:
            s = self._sentence
            self._reset()
            return s

        return None
