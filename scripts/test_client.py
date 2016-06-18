import logging
from multiprocessing.connection import Client

logging.basicConfig()
logger = logging.getLogger()
logger.setLevel(logging.DEBUG)

description = ''
spec = 'apple|banana'
choices = {}


address = ('localhost', 8000)
logger.info('making a client')
conn = Client(address)

logger.info('sending...')
conn.send(['set', description, spec, choices])

logger.info('waiting for result')
msg = conn.recv()

logger.info('received result: %s', msg)
conn.close()