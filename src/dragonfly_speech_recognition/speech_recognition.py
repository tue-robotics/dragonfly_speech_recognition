#!/usr/bin/env python
import rospy
import rospkg 

import sys, os
import time
import logging
import pythoncom

rospack = rospkg.RosPack()

def error(error):
    print "ERROR: %s"%error
    sys.exit()

try:
    dragonfly_path = "%s/deps/dragonfly"%rospack.get_path('dragonfly_speech_recognition')
    compiled_msgs_path = "%s/compiled_msgs_srvs"%rospack.get_path('dragonfly_speech_recognition')
except:
    error("Could not get package path from package dragonfly_speech_recognition")

sys.path.append(dragonfly_path)
sys.path.append(compiled_msgs_path)

try:
    from dragonfly.engines.backend_sapi5.engine import Sapi5InProcEngine
    from dragonfly import (Grammar, CompoundRule, Dictation, Choice)
except:
    error("Failed to import dragonfly, path: %s"%dragonfly_path)

try:
    from dragonfly_speech_recognition.msg._Choice import Choice as ChoiceMsg
    from dragonfly_speech_recognition.srv._GetSpeech import GetSpeech as GetSpeechSrv
except:
    error("Failed to import compiled msgs and srvs, path: %s"%compiled_msgs_path)

#---------------------------------------------------------------------------
# Set up basic logging.

logging.basicConfig(level=logging.DEBUG)
logging.getLogger("compound.parse").setLevel(logging.INFO)

class NameRule(CompoundRule):
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

def doSpeechRecognition(req):
    print req.data
    res = String()
    res.data = "omg"
    return res

def loadGrammar():
    name_rule = NameRule()
    grammar = Grammar("names and drinks choice")
    grammar.add_rule(name_rule)
    grammar.load()   

if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO)

    engine = Sapi5InProcEngine()
    engine.connect()

    loadGrammar()

    rospy.init_node('speech_recognition')
    s = rospy.Service('speech_recognition', GetSpeechSrv, doSpeechRecognition)

    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        pythoncom.PumpWaitingMessages()
        r.sleep()
