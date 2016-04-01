#!/usr/bin/env python

import sys
import os
import logging

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)

if os.name is not 'nt':
    logger.error("This module should run on a windows machine :)")
    sys.exit(1)

import time
import pythoncom
import winsound
import socket
from SimpleXMLRPCServer import SimpleXMLRPCServer as Server
from Queue import Queue, Empty
from collections import namedtuple

dragonfly_path = "%s/../../deps/dragonfly" % os.path.dirname(os.path.realpath(__file__))
data_path = "%s/../../data" % os.path.dirname(os.path.realpath(__file__))
sys.path.append(dragonfly_path)

try:
    from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
    from dragonfly import (Grammar, CompoundRule, Dictation, Choice)
except:
    logger.error("Failed to import dragonfly, path: %s" % dragonfly_path)
    sys.exit(-1)


'''Internal struct to store speech results'''
Result = namedtuple('Result', ['node', 'extras'])

def recognize(spec, choices_values, timeout):
    """
    RPC callback that will call dragonfly
    """
    logger.info('RPC request: "%s", %s (timeout %d seconds)', spec, repr(choices_values), timeout)

    try:
        result = dragonfly_recognise(spec=spec, choices_values=choices_values, timeout=timeout)
    except Exception as e:
        logger.exception('dragonfly exception:')
        raise e

    logger.info('Result: %s', repr(result))
    return result


def dragonfly_recognise(spec, choices_values, timeout):
    """
    Build a grammar based on spec and choices and send it to dragonfly
    """

    # build the dragonfly request
    extras = []
    for name, choices in choices_values.iteritems():
        extras.append(Choice(name, dict((c, c) for c in choices)))

    results = Queue()

    class GrammarRule(CompoundRule):
        def _process_recognition(self, node, extras):
            logger.info('_process_recognition callback: %s', str(node))
            results.put_nowait(Result(node=node, extras=extras))

        def _process_begin(self):
            logger.debug('Rule:__process_begin')


    rule = GrammarRule(spec=spec, extras=extras)

    grammar = Grammar("grammar")
    grammar.add_rule(rule)

    # attach failure callback
    def process_recognition_failure():
        logger.info('Grammar:process_recognition_failure')
    grammar.process_recognition_failure = process_recognition_failure

    grammar.load()
    if len(spec) > 1000: # GPS HACK
        engine.speak('Give your command after the ping')
    winsound.PlaySound(data_path + "/grammar_loaded.wav", winsound.SND_ASYNC)

    logger.info("Grammar loaded: %s", spec)

    future = time.time() + timeout
    while time.time() < future and results.empty():
        pythoncom.PumpWaitingMessages()
        time.sleep(.1)

    grammar.unload()

    try:
        result = results.get_nowait()
    except Empty:
        logger.info('No result, probably a timeout')
        return False

    if not results.empty():
        raise Exception('Multiple results received')

    # filter all extras with _ because they are private
    return {
        "result": result.node.value(),
        "choices": {k: v for (k, v) in result.extras.items() if not k.startswith('_')}
    }


def process_result():
    pass


if __name__ == "__main__":
    engine = Sapi5InProcEngine()
    engine.connect()

    address = socket.gethostbyname(socket.gethostname())
    port = 8000

    server = Server((address, port), allow_none=True)
    server.register_function(recognize, 'recognize')

    engine.speak('Speak recognition active!')
    logger.info("Speak recognition active at %s:%d", address, port)

    server.serve_forever()


