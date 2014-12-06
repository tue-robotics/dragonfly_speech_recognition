#!/usr/bin/env python

import roslib; roslib.load_manifest('dragonfly_speech_recognition')
import rospy
import rosnode

from dragonfly_speech_recognition.srv import GetSpeech
from dragonfly_speech_recognition.msg import Choice

from xmlrpclib import ServerProxy
from os import popen

def cyan(text):
    return "\x1b[00;96m%s\x1b[0m"%text

class VirtualBox():

    def __init__(self):
        pass

    def start(self):
        codes = []

        rospy.loginfo("-- [%s]"%cyan("Restoring current VM state"))
        pipe = popen('vboxmanage snapshot thespeechmachine restorecurrent')
        rospy.loginfo(pipe.read())
        codes.append(pipe.close())

        rospy.loginfo("-- [%s]"%cyan("Starting virtual machine"))
        pipe = popen('vboxmanage startvm thespeechmachine --type headless')
        rospy.loginfo(pipe.read())
        codes.append(pipe.close())

        return not any(codes)

    def __del__(self):
        print "-- [%s]"%cyan("Powering off virtual machine")
        print popen('vboxmanage controlvm thespeechmachine poweroff').read()

class GetSpeechClient():

    def __init__(self, speech_server_ip):
        self.srv = rospy.Service('~get_speech', GetSpeech, self.srv_callback)
        self.sp  = ServerProxy("http://%s:8000"%speech_server_ip)

    def srv_callback(self, req):
        spec = req.spec
        choices = dict((x.id, x.values) for x in req.choices)
        time_out = req.time_out.to_sec()

        rospy.loginfo("Speech specification: [%s]"%cyan(spec))

        # RPC request to vbox
        result = self.sp.recognize(spec, choices, time_out)
        if result:
            result["choices"] = [ Choice(id=k, values=[v]) for k, v in result["choices"].iteritems() ]
            rospy.loginfo("Speech result: [%s]"%cyan(result["result"]))
        else:
            rospy.loginfo("Speech result: [%s]"%cyan("None"))

        return result

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('get_speech_client')
        if rospy.has_param('~ip'):
            ip = rospy.get_param('~ip')
            client = GetSpeechClient(ip)
            vb = VirtualBox()
            if vb.start():
                rospy.loginfo("GetSpeech client initialized [vbox-server on %s -- booting]"%ip)
                rospy.spin()
        else:
            rospy.logerr("GetSpeech client: no virtual box ip set; please specify the local 'ip' parameter")
    except rospy.ROSInterruptException: 
        pass
