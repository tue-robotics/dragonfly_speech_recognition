#!/usr/bin/env python

import roslib; roslib.load_manifest('dragonfly_speech_recognition')
import rospy

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

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('get_speech_server_vbox')
        vb = VirtualBox()
        if vb.start():
            rospy.loginfo("GetSpeech Server initialized [booting...]")
            rospy.spin()
    except rospy.ROSInterruptException: 
        pass
