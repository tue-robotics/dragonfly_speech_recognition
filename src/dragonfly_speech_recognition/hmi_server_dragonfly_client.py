import rospy
from hmi import AbstractHMIServer, HMIResult
from dragonfly_speech_recognition.dragonfly_client import DragonflyClient


class HMIServerDragonflyClient(AbstractHMIServer):

    def __init__(self):
        """
        DragonflyHMIServer that exposes the HMI ROS Server and holds a socket client that talks to the dragonfly
        speech recognition server.
        """
        super(HMIServerDragonflyClient, self).__init__(rospy.get_name())
        self._address = (rospy.get_param('~dragonfly_server_ip', 'localhost'),
                         rospy.get_param('~dragonfly_server_port', 3000))

    def _determine_answer(self, description, grammar, target, is_preempt_requested):
        c = DragonflyClient(*self._address)
        result = c.recognize(grammar, target, is_preempt_requested)

        if result is None:
            return None

        semantics, sentence = result

        return HMIResult(sentence, semantics)
