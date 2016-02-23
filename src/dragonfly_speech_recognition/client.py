#!/usr/bin/env python

import roslib; roslib.load_manifest('dragonfly_speech_recognition')
import rospy

from dragonfly_speech_recognition.srv import GetSpeech
from dragonfly_speech_recognition.msg import Choice
from compiler.ast import flatten

from xmlrpclib import ServerProxy

def cyan(text):
    return "\x1b[00;96m%s\x1b[0m"%text

class GetSpeechClient():

    def __init__(self, speech_server_ip):
        self.srv = rospy.Service('~get_speech', GetSpeech, self.srv_callback)
        self.sp  = ServerProxy("http://%s:8000"%speech_server_ip)

    def srv_callback(self, req):
        spec = req.spec
        choices = dict((x.id, x.values) for x in req.choices)
        time_out = req.time_out.to_sec()

        rospy.loginfo("Speech specification: [%s]"%cyan(spec))

        # RPC request to server
        result = self.sp.recognize(spec, choices, time_out)
        rospy.loginfo(result)
        if result:
            if isinstance(result["result"], list):
                result["result"] = " ".join(flatten(result["result"]))
            result["choices"] = [ Choice(id=k, values=[v]) for k, v in result["choices"].iteritems() ]
            rospy.loginfo("Speech result: [%s]"%cyan(result["result"]))
        else:
            rospy.loginfo("Speech result: [%s]"%cyan("None"))
            result = {}
            result["result"] = ""
            result["choices"] = []

        return result

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('get_speech_client')
        if rospy.has_param('~ip'):
            ip = rospy.get_param('~ip')
            client = GetSpeechClient(ip)
            rospy.loginfo("GetSpeech client initialized [connecting to server on ip %s]"%ip)
            rospy.spin()
        else:
            rospy.logerr("GetSpeech client: no server ip set; please specify the local 'ip' parameter")
    except rospy.ROSInterruptException: 
        pass
