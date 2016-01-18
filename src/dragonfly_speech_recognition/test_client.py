#!/usr/bin/env python

from xmlrpclib import ServerProxy
import sys

class GetSpeechClient():

    def __init__(self, speech_server_ip):
        self.sp  = ServerProxy("http://%s:8000"%speech_server_ip)

    def call(self, spec):
        # RPC request to server
        return self.sp.recognize(spec, {}, 10)

# Main function
if __name__ == '__main__':
    ip = sys.argv[1]
    spec = sys.argv[2]
    client = GetSpeechClient(ip)

    print client.call(spec)
