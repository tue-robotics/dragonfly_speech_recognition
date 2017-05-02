#!/usr/bin/env python
from dragonfly_speech_recognition.dragonfly_client import DragonflyClient
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--ip", default="localhost", type=str)
parser.add_argument("--port", default=3000, type=int)
parser.add_argument("grammar", type=str)
parser.add_argument("root", type=str)

args = parser.parse_args()

c = DragonflyClient(args.ip, args.port)

print "Calling recognize on dragonfly server ({}:{}) for grammar:" \
      "\n\n{}\n\n" \
      "----------------------------------------------------------" \
      "\n\n{}".format(args.ip, args.port, args.grammar, c.recognize(args.grammar, args.root))
