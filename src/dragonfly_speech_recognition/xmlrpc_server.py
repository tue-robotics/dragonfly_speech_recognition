#!/usr/bin/env python

import sys
import os
import logging

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)

if os.name is not 'nt':
    #logger.error("This module should run on a windows machine :)")

    class Sapi5InProcEngine:

        def connect(self):
            pass

        def speak(self, msg):
            pass

    class CompoundRule:

        def __init__(self, spec, extras):
            pass

    class Grammar:
        def __init__(self, str):
            pass

        def add_rule(self, rule):
            pass

        def load(self):
            pass

        def unload(self):
            pass            

else:
    import pythoncom
    import winsound

    dragonfly_path = "%s/../../deps/dragonfly" % os.path.dirname(os.path.realpath(__file__))
    data_path = "%s/../../data" % os.path.dirname(os.path.realpath(__file__))
    sys.path.append(dragonfly_path)

    try:
        from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
        from dragonfly import (Grammar, CompoundRule, Dictation, Choice)
    except:
        logger.error("Failed to import dragonfly, path: %s" % dragonfly_path)
        sys.exit(-1)

import time
import socket
from SimpleXMLRPCServer import SimpleXMLRPCServer as Server
from SocketServer import ThreadingMixIn

from Queue import Queue, Empty
from collections import namedtuple

global RECOGNITION_PROCESS
RECOGNITION_PROCESS = None

'''Internal struct to store speech results'''
Result = namedtuple('Result', ['node', 'extras'])

# ------------------------------------------------------------------------------------------

class RecognitionProcess:

    def __init__(self, spec, choices_values):
        self.spec = spec
        self.choices_values = choices_values
        self.running = False

        # build the dragonfly request
        extras = []
        for name, choices in choices_values.iteritems():
            extras.append(Choice(name, dict((c, c) for c in choices)))

        self.results = Queue()

        class GrammarRule(CompoundRule):
            def _process_recognition(self, node, extras):
                logger.info('_process_recognition callback: %s', str(node))
                self.results.put_nowait(Result(node=node, extras=extras))

            def _process_begin(self):
                logger.debug('Rule:__process_begin')


        rule = GrammarRule(spec=spec, extras=extras)

        self.grammar = Grammar("grammar")
        self.grammar.add_rule(rule)

        # attach failure callback
        def process_recognition_failure():
            logger.info('Grammar:process_recognition_failure')
        self.grammar.process_recognition_failure = process_recognition_failure

        self.grammar.load()

        logger.info("Grammar loaded: %s", spec)

    def run(self, timeout):
        self.running = True

        if os.name is 'nt':
            winsound.PlaySound(data_path + "/grammar_loaded.wav", winsound.SND_ASYNC)

        self.cancel_flag = False

        future = time.time() + timeout
        while time.time() < future and self.results.empty():
            if os.name is 'nt':
                pythoncom.PumpWaitingMessages()

            if self.cancel_flag:
                self.running = False
                return {}

            time.sleep(.1)

        self.grammar.unload()

        try:
            result = self.results.get_nowait()
        except Empty:
            logger.info('No result, probably a timeout')
            self.running = False
            return False

        if not results.empty():
            self.running = False
            raise Exception('Multiple results received')

        # filter all extras with _ because they are private
        return {
            "result": result.node.value(),
            "choices": {k: v for (k, v) in result.extras.items() if not k.startswith('_')}
        }

        self.running = False

    def flag_cancel(self):
        self.cancel_flag = True

    def join(self):
        while self.running:
            time.sleep(0.1)

# ------------------------------------------------------------------------------------------

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

def cancel():
    global RECOGNITION_PROCESS
    if not RECOGNITION_PROCESS:
        return

    RECOGNITION_PROCESS.flag_cancel()
    RECOGNITION_PROCESS.join()
    RECOGNITION_PROCESS = None

def dragonfly_recognise(spec, choices_values, timeout):
    """
    Build a grammar based on spec and choices and send it to dragonfly
    """

    global RECOGNITION_PROCESS
    if RECOGNITION_PROCESS:
        RECOGNITION_PROCESS.flag_cancel()
        RECOGNITION_PROCESS.join()

        if RECOGNITION_PROCESS.spec != spec or RECOGNITION_PROCESS.choices_values != choices_values:
            RECOGNITION_PROCESS = RecognitionProcess(spec, choices_values)
    else:
        RECOGNITION_PROCESS = RecognitionProcess(spec, choices_values)

    return RECOGNITION_PROCESS.run(timeout)    

def process_result():
    pass

if __name__ == "__main__":
    engine = Sapi5InProcEngine()
    engine.connect()

    address = socket.gethostbyname(socket.gethostname())
    port = 8000

    class MyXMLRPCServer(ThreadingMixIn, Server):
        pass

    server = MyXMLRPCServer((address, port), allow_none=True)
    server.register_function(recognize, 'recognize')
    server.register_function(cancel, 'cancel')

    engine.speak('Speak recognition active!')
    logger.info("Speak recognition active at %s:%d", address, port)

    server.serve_forever()
