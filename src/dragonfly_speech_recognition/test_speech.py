#!/usr/bin/env python

'''
Check if speech works on the Windows machine
'''

import sys
from time import sleep
import logging

import os


FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logger = logging.getLogger(__name__)
logger.setLevel(logging.INFO)


def error(msg, *args, **kwargs):
    logger.error(msg, *args, **kwargs)
    sys.exit(1)


if os.name is not 'nt':
    print "This module should run on a windows machine :)"
    sys.exit()

import pythoncom

current_dir = os.path.dirname(os.path.realpath(__file__))
dragonfly_path = os.path.join(current_dir, '..', '..', 'deps', 'dragonfly')
sys.path.append(dragonfly_path)

try:
    from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
    from dragonfly import (Grammar, CompoundRule, Dictation, Choice)
except:
    error("Failed to import dragonfly, path: %s" % dragonfly_path)


engine = Sapi5InProcEngine()
engine.connect()
engine.speak('Speak recognition active!')


# Voice command rule combining spoken form and recognition processing.
class ExampleRule(CompoundRule):
    spec = "do something computer"  # Spoken form of command.

    def _process_recognition(self, node, extras):  # Callback when command is spoken.
        print "Voice command spoken."

# Create a grammar which contains and loads the command rule.
grammar = Grammar("example grammar")  # Create a grammar to contain the command    rule.
grammar.add_rule(ExampleRule())  # Add the command rule to the grammar.
logger.info("Loading Grammar")
grammar.load()  # Load the grammar.
logger.info("Grammar loaded")

while True:
    pythoncom.PumpWaitingMessages()
    sleep(.1)

grammar.unload()