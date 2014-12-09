#!/usr/bin/env python

import roslib; roslib.load_manifest('dragonfly_speech_recognition')
import rospy
import rosnode
import random
import re

from dragonfly_speech_recognition.srv import GetSpeech
from dragonfly_speech_recognition.msg import Choice

def cyan(text):
    return "\x1b[00;96m%s\x1b[0m"%text

class GetSpeechDummyClient():

    def __init__(self):
        self.srv = rospy.Service('~get_speech', GetSpeech, self.srv_callback)

    def srv_callback(self, req):
        result = {}

        # Copy request
        result["choices"] = req.choices
        result["result"] = req.spec

        # Choose random choice :)
        [ random.shuffle(c.values) for c in req.choices ]
        
        # Parse the choices in the ending result :)
        for c in result["choices"]:
            result["result"] = result["result"].replace("<%s>"%c.id, c.values[0]) 

        # Check if we have no more choices in result, otherwise exception is thrown
        if re.match(".*<[^<>]+>.*", result["result"]):
            rospy.logerr("Not all choices could be resolved in the specification, residual result: '%s'"%result["result"])
            return False

        rospy.loginfo("Speech specification: [%s]"%cyan(req.spec))
        rospy.loginfo("Speech result: [%s]"%cyan(result["result"]))

        return result

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('get_speech_client')
        client = GetSpeechDummyClient()
        rospy.loginfo("GetSpeechDummy client initialized")
        rospy.spin()
    except rospy.ROSInterruptException: 
        pass
