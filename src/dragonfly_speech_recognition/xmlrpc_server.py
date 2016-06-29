#!/usr/bin/env python

import sys
import os
import logging
import threading

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

'''Internal struct to store speech results'''
Result = namedtuple('Result', ['node', 'extras'])

# ------------------------------------------------------------------------------------------

class RecognitionProcess:

    def __init__(self, spec, choices_values):
        self.spec = spec
        self.choices_values = choices_values
        
        self.thread = None
        self.cancel_flag = False

    def run_threaded(self):
        self.results = Queue()        
        self.thread = threading.Thread(target=self._run)
        self.thread.start()

    def _run(self):
        self.cancel_flag = False

        # build the dragonfly request
        extras = []
        for name, choices in self.choices_values.iteritems():
            extras.append(Choice(name, dict((c, c) for c in choices)))

        class GrammarRule(CompoundRule):
            def _process_recognition(self, node, extras):
                logger.info('_process_recognition callback: %s', str(node))
                self.results.put_nowait(Result(node=node, extras=extras))

            def _process_begin(self):
                logger.debug('Rule:__process_begin')


        rule = GrammarRule(spec=self.spec, extras=extras)
        rule.results = self.results

        self.grammar = Grammar("grammar")
        self.grammar.add_rule(rule)

        # attach failure callback
        def process_recognition_failure():
            logger.info('Grammar:process_recognition_failure')
        self.grammar.process_recognition_failure = process_recognition_failure

        self.grammar.load()

        logger.info("Grammar loaded: %s", self.spec)

        # - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - - -

        if os.name is 'nt':
            winsound.PlaySound(data_path + "/grammar_loaded.wav", winsound.SND_ASYNC)

        timeout = 3600

        future = time.time() + timeout

        while time.time() < future:
            if os.name is 'nt':
                pythoncom.PumpWaitingMessages()

            if self.cancel_flag:
                return

            time.sleep(.1)


    def get_result(self):

        if self.results.empty():
            return None

        try:
            result = self.results.get_nowait()
        except Empty:
            logger.info('No result, probably a timeout')
            return None

        if not self.results.empty():
            raise Exception('Multiple results received')

        # filter all extras with _ because they are private
        return {
            "result": result.node.value(),
            "choices": {k: v for (k, v) in result.extras.items() if not k.startswith('_')}
        }

    def stop(self):
        if not self.thread:
            return

        self.cancel_flag = True
        self.thread.join()
        self.thread = None

    def is_running(self):
        return self.thread != None

# ------------------------------------------------------------------------------------------

class SpeechServer(ThreadingMixIn, Server):

    def register(self):
        self.register_function(self.recognize, 'recognize')
        self.register_function(self.cancel, 'cancel')
        self.process = None

        self.handling_request = False
        self.flag_request_abort = False

    def recognize(self, spec, choices_values, timeout):
        """
        RPC callback that will call dragonfly
        """
        logger.info('RPC request: "%s", %s (timeout %d seconds)', spec, repr(choices_values), timeout)

        try:
            result = self.dragonfly_recognise(spec=spec, choices_values=choices_values, timeout=timeout)
        except Exception as e:
            logger.exception('dragonfly exception:')
            raise e

        logger.info('Result: %s', repr(result))
        return result

    def cancel(self):
        if not self.process:
            return
        self.flag_request_abort = True

    def dragonfly_recognise(self, spec, choices_values, timeout):
        """
        Build a grammar based on spec and choices and send it to dragonfly
        """

        if self.handling_request:
            self.flag_request_abort = True
            while self.handling_request:
                time.sleep(0.1)

        self.flag_request_abort = False
        self.handling_request = True

        if not self.process or self.process.spec != spec or self.process.choices_values != choices_values:
            if self.process:
                self.process.stop()
            self.process = RecognitionProcess(spec, choices_values)
            self.process.run_threaded()       

        while self.process.is_running() and not self.flag_request_abort:
            print "Recognizing, process = {}".format(self.process)
            result = self.process.get_result()
            if result:
                return result
            time.sleep(0.1)

        self.handling_request = False

    def process_result(self):
        pass

if __name__ == "__main__":
    engine = Sapi5InProcEngine()
    engine.connect()

    address = socket.gethostbyname(socket.gethostname())
    port = 8000

    server = SpeechServer((address, port), allow_none=True)
    server.register()

    engine.speak('Speak recognition active!')
    logger.info("Speak recognition active at %s:%d", address, port)

    try:
        server.serve_forever()
    except KeyboardInterrupt:
        if server.process:
            server.process.stop()