#!/usr/bin/env python

print "omg"

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

print "Importing dragonfly"
from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
from dragonfly import Alternative, Sequence, Literal, Grammar, Rule, Optional, Repetition
import pythoncom


if __name__ == "__main__":

    print "Connected to Sapi5 InProcEngine"

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
    # beverage = Alternative([Literal('coke'), Literal('fanta')])
    # food1 = Alternative([Literal('apple'), Literal('banana')])
    # food2 = Alternative([Literal('apple'), Literal('banana')])
    # food = Sequence([food1, Literal('and'), Optional(Alternative([Literal('a'), Literal('an')])), food2])
    # seq = Alternative([beverage, food])

    # This holds 'food and beverage' or 'beverage and food'
    beverage = Alternative([Literal('coke'), Literal('fanta')])
    food = Alternative([Literal('apple'), Literal('banana')])
    s1 = Sequence([beverage, Literal('and'), food])
    s2 = Sequence([food, Literal('and'), beverage])
    seq = Alternative([s1, s2])

    # This holds all possibilities (so also 'beverage and beverage')
    # beverage = Alternative([Literal('coke'), Literal('fanta')])
    # food = Alternative([Literal('apple'), Literal('banana')])
    # a1 = Alternative([food, beverage])
    # a2 = Alternative([food, beverage])
    # seq = Sequence([a1, Literal('and'), a2])

    bla = Grammar("G")
    class CustomRule(Rule):
       def process_recognition(self, node):
           print node
           self.grammar._process_recognition(node)

    rule = CustomRule(element=seq, exported=True)
    # rule2 = CustomRule(element=RuleRef(rule))
    bla.add_rule(rule=rule)

    print "Loading grammar"
    bla.load()

    print "Going to recognize"

    # i = 0
    while True:
        # i += 1
        # print i
        # if i > 10:
        #     break

        pythoncom.PumpWaitingMessages()


    print "Recognition done"
