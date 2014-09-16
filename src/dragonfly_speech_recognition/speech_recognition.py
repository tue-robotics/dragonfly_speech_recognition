#!/usr/bin/env python
import rospy
import rospkg
import sys, os

rospack = rospkg.RosPack()

def error(error):
    print "ERROR: %s"%error
    sys.exit()

try:
    dragonfly_path = "%s/deps/dragonfly"%rospack.get_path('dragonfly_speech_recognition')
except:
    error("Could not get package path from package dragonfly_speech_recognition")

try:
    sys.path.append(dragonfly_path)
    import dragonfly
except:
    error("Failed to import dragonfly, path: %s"%dragonfly_path)

print "[[[ Succesfully loaded dragonfly ]]]"
