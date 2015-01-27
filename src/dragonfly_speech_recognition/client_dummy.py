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
        result["result"] = "(%s)" % req.spec
        result["choices"] = []

        # Pick random group if available
        while re.search('\([^\)]+\)', result["result"]):
            options = re.findall('\([^\(\)]+\)', result["result"])
            for option in options:
                result["result"] = result["result"].replace(option,random.choice(option[1:-1].split("|")), 1)

        # Fetch all the residual choices
        choices = re.findall("<([^<>]+)>", result["result"])
        
        # Parse the choices in the ending result :)
        for c in choices:
            for req_c in req.choices:
                if req_c.id == c:
                    value = random.choice(req_c.values)

                    result["result"] = result["result"].replace("<%s>"%c, value) 
                    result["choices"].append(Choice(id=c, values=[value])) 

        # Remove the optional brackets []
        result["result"] = result["result"].replace("[","").replace("]","")

        # Check if result is clean
        if re.match(".*[<>\(\)\[\]]+.*", result["result"]):
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
