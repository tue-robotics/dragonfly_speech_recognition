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

class GrammarRule(CompoundRule):
    spec = "(My name is|I am) <name>"
    extras = [Choice("name",   {
                                    "Michael":"Michael",
                                    "Cristopher":"Cristopher",
                                    "Matthew":"Matthew",
                                    "Joshua":"Joshua",
                                    "Daniel":"Daniel",
                                    "David":"David",
                                    "Andrew":"Andrew",
                                    "James":"James",
                                    "Justin":"Justin",
                                    "Joseph":"Joseph",
                                    "Jessica":"Jessica",
                                    "Ashley":"Ashley",
                                    "Brittany":"Brittany",
                                    "Amanda":"Amanda",
                                    "Samantha":"Samantha",
                                    "Sarah":"Sarah",
                                    "Stephanie":"Stephanie",
                                    "Jennifer":"Jennifer",
                                    "Elizabeth":"Elizabeth",
                                    "Lauren":"Lauren",
                                }
                     )
              ]
    
    def _process_recognition(self, node, extras):
        print extras["name"]

def loadGrammar():
    grammar = Grammar("grammar")
    grammar.add_rule(GrammarRule())
    grammar.load()   

def serverThread():
    server = Server(("localhost", 8000))
    print "Listening on port 8000..."
    server.serve_forever()

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    engine = Sapi5InProcEngine()
    engine.connect()

    loadGrammar()

    # Start server thread
    t = Thread(target=serverThread)
    t.start()

    engine.speak('Speak recognition active!')

    while 1:
        pythoncom.PumpWaitingMessages()
        time.sleep(.1)
