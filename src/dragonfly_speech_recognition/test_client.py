#!/usr/bin/env python

from xmlrpclib import ServerProxy, Error, Fault
import sys
import logging

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
            return self.sp.recognize('my name is <name>', {'name': ['Ramon', 'Paul']}, 10)
        except Fault as error:
            logger.exception('RPC Fault (%d): %s', error.faultCode, error.faultString)
            sys.exit(1)


# Main function
if __name__ == '__main__':
    ip = sys.argv[1]
    spec = sys.argv[2]
    client = GetSpeechClient(ip)

    print client.call(spec)
