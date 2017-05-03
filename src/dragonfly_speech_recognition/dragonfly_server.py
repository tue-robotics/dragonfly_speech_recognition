#!/usr/bin/env python
import logging
import multiprocessing.connection
# from threading import Thread

FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)

# Try to import the dragonfly wrapper, if it fails (on a unix machine), use the stub instead
try:
    from dragonfly_wrapper import DragonflyWrapper
except ImportError as e:
    logger.warn("Failed to import DragonflyWrapper {} using stub instead ...".format(e))
    from dragonfly_wrapper_stub import DragonflyWrapper


class DragonflyServer:
    def __init__(self, ip, port):
        self._listener = multiprocessing.connection.Listener((ip, port))
        self._dragonfly_wrapper = DragonflyWrapper()

    def spin(self):
        while True:
            try:
                conn = self._listener.accept()
                # t = Thread(target=self._process_connection, args=[conn])
                # t.start()
                # t.join()
                self._process_connection(conn)
            except KeyboardInterrupt:
                logger.warn('keyboard interrupt')
                break
            except Exception:
                logger.exception('speech exception')

            conn.close()

    def _process_connection(self, conn):
        grammar, target = conn.recv()
        logger.info('Connection accepted from {}\n'
                    ' - Grammar \n\n {} \n\n - Target \n\n {}'.format(self._listener.last_accepted, grammar, target))

        self._dragonfly_wrapper.set_grammar(grammar, target)

        print "Grammar set"

        while True:
            if conn.poll(.1):  # We have received a cancel request
                self._dragonfly_wrapper.unset_grammar()
                break
            recognition = self._dragonfly_wrapper.get_recognition()
            if recognition is not None:
                conn.send(recognition)
                break
