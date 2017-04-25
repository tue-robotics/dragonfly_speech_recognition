#!/usr/bin/env python

import argparse
from cfgparser import CFGParser
import re
import random
import yaml

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

engine = Sapi5InProcEngine()
engine.connect()

import time

arg_parser = argparse.ArgumentParser()
arg_parser.add_argument("--grammar", type=str, required=True)
args = arg_parser.parse_args()


def random_fold_options(spec):
    """
    When you have a spec with options, just randomly choose them
    """
    # Pick random group if available
    spec = "(%s)" % spec
    while re.search('\([^)]+\)', spec):
        options = re.findall('\([^()]+\)', spec)
        for option in options:
            spec = spec.replace(option, random.choice(option[1:-1].split("|")), 1)

    return spec

# Mapping from name to Alternative, Sequence or Literal
RULES = {}


def custom(lname, parser, depth=0):
    if lname not in parser.rules:
        return None

    rule = parser.rules[lname]
    print rule

    option_alternative_list = []
    for opt in rule.options:

        if len(opt.conjuncts) == 1 and not opt.conjuncts[0].is_variable:
            option_alternative_list.append(Literal(opt.conjuncts[0].name))
            continue

        conjunctions_list = []

        for conj in opt.conjuncts:

            if conj.is_variable:
                result = custom(conj.name, parser, depth + 1)
                if result:
                    conjunctions_list.append(result)
            else:
                conjunctions_list.append(Literal(conj.name))

        if len(conjunctions_list) == 1:
            option_alternative_list.append(conjunctions_list[0])
        else:
            option_alternative_list.append(Sequence(conjunctions_list))

    if len(option_alternative_list) == 1:
        return option_alternative_list[0]

    return Alternative(option_alternative_list)


def unwrap_grammar(lname, parser):
    if lname not in parser.rules:
        return ""

    rule = parser.rules[lname]

    opt_strings = []
    for opt in rule.options:
        conj_strings = []

        for conj in opt.conjuncts:

            if conj.is_variable:
                unwrapped_string = unwrap_grammar(conj.name, parser)
                if unwrapped_string:
                    conj_strings.append(unwrapped_string)
            else:
                conj_strings.append(conj.name)

        opt_strings.append(" ".join(conj_strings))

    s = "|".join(opt_strings)

    if len(opt_strings) > 1:
        s = "(" + s + ")"

    return s


def parse(cfg_parser, root, sentence):
    semantics_str = cfg_parser.parse(root, sentence.split(" "))
    semantics_str = semantics_str.replace("<", "[")
    semantics_str = semantics_str.replace(">", "]")

    return yaml.load(semantics_str)


cfg_parser = CFGParser.fromfile(args.grammar)

root = "T"
#unwrapped = unwrap_grammar(root, cfg_parser)
#random_answer = random_fold_options(unwrapped)
element = custom(root, cfg_parser)

#print "Unwrapped:", unwrapped
#print "Random answer:", random_answer
#print "Parsed:", parsed

bla = Grammar("G")
from compiler.ast import flatten
import logging

logging.basicConfig()
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)

RESULT = None

class CustomRule(Rule):

   def process_recognition(self, node):
       result = node.value()
       flattened_string = " ".join(flatten(result))
       print "flattened string:", flattened_string
       parsed = parse(cfg_parser, root, flattened_string)
       print "parsed:", parsed


rule = CustomRule(element=element, exported=True)
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
