#!/usr/bin/env python

import sys, os
import time
import logging
import pythoncom

# XML RPC SERVER
from threading import Thread
import xmlrpclib
from SimpleXMLRPCServer import SimpleXMLRPCServer as Server

def error(error):
    print "ERROR: %s"%error
    sys.exit()

dragonfly_path = "%s/../../deps/dragonfly"%os.path.dirname(os.path.realpath(__file__))
sys.path.append(dragonfly_path)

try:
    from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
    from dragonfly import (Grammar, CompoundRule, Dictation, Choice)
except:
    error("Failed to import dragonfly, path: %s"%dragonfly_path)

#---------------------------------------------------------------------------

logging.basicConfig(level=logging.DEBUG)
logging.getLogger("compound.parse").setLevel(logging.INFO)

#---------------------------------------------------------------------------

RESULT = None
ENGINE = None

#---------------------------------------------------------------------------

class GrammarRule(CompoundRule):  
    spec = "test"
    extras = []  
    def _process_recognition(self, node, extras):
        global RESULT

        RESULT = extras
        print extras["name"]

# RPC METHOD
def recognize(spec, extras):
    global RESULT

    print RESULT
    RESULT = None

    print "Recognizing: ", spec, extras
    print ENGINE
    print dir(ENGINE)

    grammar = Grammar("grammar")
    rule = GrammarRule()
    rule.spec = spec
    rule.extras = []
    for name, choices in extras.iteritems():
        rule.extras.append(Choice(name, dict((c,c) for c in choices)))
    grammar.add_rule(rule)
    grammar.load()   

def serverThread():
    server = Server(("localhost", 8000))
    server.register_function(recognize, 'recognize')
    server.serve_forever()

logging.basicConfig(level=logging.INFO)

ENGINE = Sapi5InProcEngine()
ENGINE.connect()

recognize("Just call me <name>", {"name":["Michael","Cristopher","Matthew","Joshua","Daniel","David","Andrew","James","Justin","Joseph","Jessica","Ashley","Brittany","Amanda","Samantha","Sarah","Stephanie","Jennifer","Elizabeth","Lauren"]})

# Start server thread
t = Thread(target=serverThread)
t.start()

ENGINE.speak('Speak recognition active!')

while 1:
    pythoncom.PumpWaitingMessages()
    time.sleep(.1)

t.stop()
