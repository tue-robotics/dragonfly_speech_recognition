#!/usr/bin/env python
from dragonfly_speech_recognition.dragonfly_client import DragonflyClient
import argparse
import os


def read_valid_file(p, arg):
    if not os.path.exists(arg):
        p.error("The file %s does not exist!" % arg)
    else:
        return open(arg, 'r').read()

parser = argparse.ArgumentParser()
parser.add_argument("target", type=str)
parser.add_argument("--ip", default="localhost", type=str)
parser.add_argument("--port", default=3000, type=int)
parser.add_argument("--grammar", type=str)
parser.add_argument("--grammar-file", type=lambda x: read_valid_file(parser, x))
args = parser.parse_args()

# Verify the specified grammar
if (args.grammar_file and args.grammar) or (args.grammar_file is None and args.grammar is None):
    parser.error("Please specify either a grammar string using --grammar of a grammar file using --grammar-file")
grammar = args.grammar_file if args.grammar_file else args.grammar

args = parser.parse_args()

c = DragonflyClient(args.ip, args.port)

print "Calling recognize on dragonfly server ({}:{}) for grammar:" \
      "\n\n{}\n\n" \
      "----------------------------------------------------------" \
      "\n\n{}".format(args.ip, args.port, grammar, c.recognize(grammar, args.target))
