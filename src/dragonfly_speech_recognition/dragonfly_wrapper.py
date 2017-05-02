import logging

import dassdadasdas

FORMAT = '%(asctime)s %(module)s [%(levelname)s] %(message)s'
logging.basicConfig(format=FORMAT)
logging.getLogger().setLevel(logging.DEBUG)
logger = logging.getLogger(__name__)

# make dragonfly less verbose
logging.getLogger('engine.compiler').setLevel(logging.INFO)
logging.getLogger('grammar.decode').setLevel(logging.INFO)
logging.getLogger('grammar.begin').setLevel(logging.INFO)
logging.getLogger('compound.parse').setLevel(logging.INFO)


class DragonflyWrapper:
    def __init__(self):
        pass

    def set_grammar(self, grammar):
        pass

    def unset_grammar(self):
        pass

    def get_recognition(self):
        return "Blaat"

