#!/usr/bin/env python

'''
Tooling to start up the VirtualBox from ROS
'''

import roslib; roslib.load_manifest('dragonfly_speech_recognition')
import rospy

import subprocess
import sys
import os

def cyan(text):
    return "\x1b[00;96m%s\x1b[0m"%text

class VirtualBox():

    def __init__(self):
        pass

    def start(self):
        codes = []

        rospy.loginfo("-- [%s]"%cyan("Restoring current VM state"))
        codes.append(subprocess.call(['vboxmanage', 'snapshot', 'thespeechmachine', 'restorecurrent']))

        rospy.loginfo("-- [%s]"%cyan("Starting virtual machine"))
        codes.append(os.system("vboxmanage startvm thespeechmachine --type headless 2>&1 | if grep -q error; then exit 1; else exit 0; fi"))

        return not any(codes)

    def __del__(self):
        print "-- [%s]"%cyan("Powering off virtual machine")
        subprocess.call(['vboxmanage', 'controlvm', 'thespeechmachine', 'poweroff'])

# Main function
if __name__ == '__main__':
    try:
        rospy.init_node('get_speech_server_vbox')
        vb = VirtualBox()
        if vb.start():
            rospy.loginfo("GetSpeech Server initialized [booting...]")
            rospy.spin()
        else:
            rospy.logerr("Failed to start virtual machine, Please recompile the kernel module and install it by 'sudo /etc/init.d/vboxdrv setup'")
            sys.exit(1)
    except rospy.ROSInterruptException:
        pass
