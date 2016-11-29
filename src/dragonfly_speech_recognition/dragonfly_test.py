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

from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
from dragonfly import *
import pythoncom

if __name__ == "__main__":

    engine = Sapi5InProcEngine()
    engine.connect()

    import time
    time.sleep(1.0)

    # seq = Sequence([Literal('banana'), Literal('apple')], name='blaat')
    # seq = Sequence([Literal('banana'), Literal('apple')])
    # seq = Alternative([Literal('banana'), Literal('apple')])
    # seq = Literal('banana')

    # s1 = Alternative([Literal('banana'), Literal('apple')])
    # s2 = Alternative([Literal('beer'), Literal('coke')])
    # s3 = Alternative([s1, s2])
    # seq = Repetition(s3, 1, 4)

    # Restaurant check: <beverage>|<food> and [(a|and)] <food>
    beverage = Alternative([Literal('coke'), Literal('fanta')])
    food1 = Alternative([Literal('apple'), Literal('banana')])
    food2 = Alternative([Literal('apple'), Literal('banana')])
    food = Sequence([food1, Literal('and'), Optional(Alternative([Literal('a'), Literal('an')])), food2])
    seq = Alternative([beverage, food])

    bla = Grammar("G")
    class CustomRule(Rule):
       def process_recognition(self, node):
           print node
           self.grammar._process_recognition(node)

    rule = CustomRule(element=seq, exported=True)
    # rule2 = CustomRule(element=RuleRef(rule))
    bla.add_rule(rule=rule)

    bla.load()

    print "Going to recognize"
    while True:
        pythoncom.PumpWaitingMessages()
    print "Recognition done"
