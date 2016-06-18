#!/usr/bin/env python

'''
Easy make a speech RPC call to an ip
'''

from xmlrpclib import ServerProxy, Error, Fault
import sys
import logging
from argparse import ArgumentParser

logging.basicConfig()
logging.getLogger().setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)


class GetSpeechClient():
    def __init__(self, speech_server_ip):
        self.sp = ServerProxy("http://%s:8000" % speech_server_ip)

    def call(self, spec):
        # RPC request to server
        logger.info('speech request: "%s"', spec)
        try:
            # return self.sp.recognize(spec, {}, 10)
            return self.sp.recognize(spec, {}, 10)
        except Fault as error:
            logger.exception('RPC Fault (%d): %s', error.faultCode, error.faultString)
            sys.exit(1)


# Main function
if __name__ == '__main__':
    parser = ArgumentParser(description="Client to send a spec to the speech server")
    parser.add_argument('IP', help="Server's IP")
    parser.add_argument('spec', help="Speech spec")
    args = parser.parse_args()


    ip = sys.argv[1]
    spec = sys.argv[2]
    client = GetSpeechClient(ip)

    print client.call(spec)
