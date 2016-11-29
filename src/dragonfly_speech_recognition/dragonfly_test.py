#!/usr/bin/env python

"""
Wraps dragonfly in a simple API
"""
from Queue import Queue, Empty
from collections import namedtuple
import logging
import sys
import time
#import winsound

import os


#logger = logging.getLogger(__name__)


def error(msg, *args, **kwargs):
    logger.error(msg, *args, **kwargs)
    sys.exit(1)


# current_dir = os.path.dirname(os.path.realpath(__file__))
# data_path = os.path.join(current_dir, '..', '..', 'data')
#
#
# def add_deps_to_path(name):
#    path = os.path.join(current_dir, '..', '..', 'deps', name)
#    sys.path.append(path)
#
#
# add_deps_to_path('dragonfly')
from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
# from dragonfly import Grammar, CompoundRule, Choice, Rule, Literal, Sequence
# from dragonfly.test import ElementTester

from dragonfly import *
from dragonfly.test import ElementTester
import pythoncom

if __name__ == "__main__":

    engine = Sapi5InProcEngine()
    engine.connect()

    import time
    time.sleep(1.0)

    # seq = Sequence([Literal('banana'), Literal('apple')], name='blaat')
    # seq = Sequence([Literal('banana'), Literal('apple')])
    # seq = Alternative([Literal('banana'), Literal('apple')])
    seq = Literal('banana')

    bla = Grammar("G")
    class CustomRule(Rule):
       def process_recognition(self, node):
           print node
           self.grammar._process_recognition(node)

    bla.add_rule(CustomRule(element=seq, exported=True))

    bla.load()

    print "Going to recognize"
    while True:
        pythoncom.PumpWaitingMessages()

    print test_seq.recognize('banana apple')
    # print test_seq.recognize('banana')
    print "Recognition done"


    #raw_input("Press any key to close")
