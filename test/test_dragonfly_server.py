#!/usr/bin/env python
from dragonfly_speech_recognition.dragonfly_server import DragonflyServer
import argparse


parser = argparse.ArgumentParser()
parser.add_argument("--ip", default="localhost", type=str)
parser.add_argument("--port", default=3000, type=int)

args = parser.parse_args()

print "Setting up DragonflyServer {}:{}".format(args.ip, args.port)
c = DragonflyServer(args.ip, args.port)

print "Spinning ..."
c.spin()
