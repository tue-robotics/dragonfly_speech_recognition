#!/usr/bin/env python

import sys, os

if os.name is not 'nt':
    print "This module should run on a windows machine :)"
    sys.exit()

import time
import logging
import pythoncom
import winsound
import socket

# XML RPC SERVER
from threading import Thread
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer as Server

def error(error):
    print "ERROR: %s"%error
    sys.exit()

dragonfly_path = "%s/../../deps/dragonfly"%os.path.dirname(os.path.realpath(__file__))
data_path = "%s/../../data"%os.path.dirname(os.path.realpath(__file__))
sys.path.append(dragonfly_path)

try:
    from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
    from dragonfly import (Grammar, CompoundRule, Dictation, Choice)
except:
    error("Failed to import dragonfly, path: %s"%dragonfly_path)

#---------------------------------------------------------------------------

RESULT = None

#---------------------------------------------------------------------------

class GrammarRule(CompoundRule):   
    def _process_recognition(self, node, extras):
        global RESULT
        RESULT = extras

# RPC METHOD
def recognize(spec, choices_values, timeout):

    # Flush waiting messages
    pythoncom.PumpWaitingMessages()
    global RESULT
    RESULT = None

    grammar = Grammar("grammar")

    extras = []
    for name, choices in choices_values.iteritems():
        extras.append(Choice(name, dict((c,c) for c in choices)))

    Rule = type("Rule", (GrammarRule,),{"spec": spec, "extras": extras})
    grammar.add_rule(Rule())
    grammar.load()

    print "Grammar loaded"
    winsound.PlaySound(data_path + "/grammar_loaded.wav", winsound.SND_ASYNC)

    future = time.time() + timeout
    while time.time() < future and RESULT is None:
        pythoncom.PumpWaitingMessages()
        time.sleep(.1)

    grammar.unload()

    result = None
    if RESULT:
        result = {}
        result["result"] = " ".join(RESULT["_node"].words())
        result["choices"] = {}
        for choice in choices_values:
            if choice in RESULT:
                result["choices"][choice] = RESULT[choice]

    return result

if __name__ == "__main__":
    engine = Sapi5InProcEngine()
    engine.connect()

    address = socket.gethostbyname(socket.gethostname())
    port = 8000

    server = Server((address, port), allow_none=True)
    server.register_function(recognize, 'recognize')

    engine.speak('Speak recognition active!')
    print "Speak recognition active at %s:%d"%(address,port)

    server.serve_forever()


