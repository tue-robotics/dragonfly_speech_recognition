#!/usr/bin/env python

"""
Wraps dragonfly in a simple API
"""
from Queue import Queue, Empty
from collections import namedtuple
import logging
import sys
import time
import winsound

import os


logger = logging.getLogger(__name__)


def error(msg, *args, **kwargs):
    logger.error(msg, *args, **kwargs)
    sys.exit(1)


current_dir = os.path.dirname(os.path.realpath(__file__))
data_path = os.path.join(current_dir, '..', '..', 'data')


def add_deps_to_path(name):
    path = os.path.join(current_dir, '..', '..', 'deps', name)
    sys.path.append(path)


try:
    import pythoncom
except ImportError as e:
    if os.name is not 'nt':
        raise RuntimeError("This module should run on a windows machine :)")
    else:
        raise e

add_deps_to_path('dragonfly')
from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
from dragonfly import Grammar, CompoundRule, Choice

sys.path.pop()

'''Internal struct to store speech results'''
Result = namedtuple('Result', ['node', 'extras'])


class DragonflyWrapper(object):
    def __init__(self):
        engine = Sapi5InProcEngine()
        engine.connect()

        engine.speak('Speak recognition active!')
        logger.info('Speak recognition active!')

        self._results = Queue()

        self._grammar = Grammar("grammar")

        # attach failure callback
        def process_recognition_failure():
            logger.info('Grammar:process_recognition_failure')

        self._grammar.process_recognition_failure = process_recognition_failure

    def set_grammar(self, spec, choices_values):
        # TODO: cache the rule
        assert self._results.empty()

        rule = self._make_rule(spec, choices_values, self._result_callback)
        self._grammar.add_rule(rule)

        self._grammar.load()
        winsound.PlaySound(data_path + "/grammar_loaded.wav", winsound.SND_ASYNC)

        logger.info("Grammar loaded: %s", spec)

    def spin_once(self):
        pythoncom.PumpWaitingMessages()

    @property
    def results(self):
        return self._results

    @staticmethod
    def _make_rule(spec, choices_values, callback):
        # build the dragonfly request
        extras = []
        for name, choices in choices_values.iteritems():
            extras.append(Choice(name, dict((c, c) for c in choices)))

        class CustomRule(CompoundRule):
            def _process_recognition(self, node, extras):
                logger.info('_process_recognition callback: %s', str(node))
                callback(node, extras)

            def _process_begin(self):
                logger.debug('Rule:__process_begin')

        rule = CustomRule(spec=spec, extras=extras)
        return rule

    def _result_callback(self, node, extras):
        self._results.put_nowait(Result(node=node, extras=extras))


if __name__ == "__main__":
    logging.basicConfig()
    logging.getLogger().setLevel(logging.INFO)

    dragon = DragonflyWrapper()
    dragon.set_grammar('my name is <name>', {
        'name': ['apple', 'jake']
    })

    future = time.time() + 10
    while time.time() < future and dragon.results.empty():
        time.sleep(.1)
        dragon.spin_once()

    try:
        result = dragon.results.get_nowait()
    except Empty:
        sys.exit('no result')

    if not dragon.results.empty():
        raise Exception('Multiple results received')

    # filter all extras with _ because they are private
    result = {
        "result": result.node.value(),
        "choices": {k: v for (k, v) in result.extras.items() if not k.startswith('_')}
    }

    print result