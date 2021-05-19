#!/usr/bin/env python
import logging
import multiprocessing.connection
import traceback

FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.INFO)
logger = logging.getLogger(__name__)

# Try to import the dragonfly wrapper, if it fails (on a unix machine), use the stub instead
try:
    from .dragonfly_wrapper import DragonflyWrapper
except ImportError as e:
    logger.warning(f"Failed to import DragonflyWrapper: '{e}' using stub instead ...\n{traceback.format_exc()}")
    from .dragonfly_wrapper_stub import DragonflyWrapper


class bcolors:
    HEADER = '\033[95m'
    OKBLUE = '\033[94m'
    OKGREEN = '\033[92m'
    WARNING = '\033[93m'
    FAIL = '\033[91m'
    ENDC = '\033[0m'
    BOLD = '\033[1m'
    UNDERLINE = '\033[4m'


class DragonflyServer:
    def __init__(self, ip, port):
        logger.info("Setting up DragonflyServer {}:{}".format(ip, port))
        self._listener = multiprocessing.connection.Listener((ip, port))
        self._dragonfly_wrapper = DragonflyWrapper()

    def spin(self):
        logger.info("Spinning ....")
        while True:
            try:
                conn = self._listener.accept()
                self._process_connection(conn)
            except KeyboardInterrupt:
                logger.warning('keyboard interrupt')
                break
            except Exception as e:
                logger.exception(f'Speech exception: {e}\n{traceback.format_exc()}')

            conn.close()

    def _process_connection(self, conn):
        # parse command
        cmd = conn.recv()
        if cmd == 'restart_node':
            logger.warning('restarting node now!!!')
            exit(1)
        grammar, target = cmd

        logger.info('Connection accepted from {} :: Target: {}'.format(self._listener.last_accepted, target))
        logger.debug("Grammar: {}".format(grammar))

        self._dragonfly_wrapper.set_grammar(grammar, target)

        while True:
            if conn.poll(.1):  # We have received a cancel request
                self._dragonfly_wrapper.unset_grammar()
                break
            recognition = self._dragonfly_wrapper.get_recognition()
            if recognition is not None:
                logging.info("Sending result back to the client: \x1b[;44m'%s'\x1b[0m", recognition)
                self._dragonfly_wrapper.unset_grammar()
                conn.send(recognition)
                break
