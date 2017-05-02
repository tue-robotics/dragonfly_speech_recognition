import rospy
import multiprocessing.connection
from hmi_server.abstract_server import AbstractHMIServer, HMIResult


class HMIServerDragonflyClient(AbstractHMIServer):

    def __init__(self):
        """
        DragonflyHMIServer that exposes the HMI ROS Server and holds a socket client that talks to the dragonfly
        speech recognition server.
        """
        super(HMIServerDragonflyClient, self).__init__(rospy.get_name())
        self._address = (rospy.get_param('~dragonfly_server_ip', 'localhost'),
                         rospy.get_param('~dragonfly_server_port', 3000))
        rospy.loginfo('Testing connection to to server {}'.format(self._address))
        conn = multiprocessing.connection.Client(self._address)
        rospy.loginfo("Success")
        conn.close()

    def _determine_answer(self, description, grammar, is_preempt_requested):
        # Set-up connection and set grammar
        conn = multiprocessing.connection.Client(self._address)
        conn.send(grammar)

        rospy.loginfo('Waiting for result ..')
        while not conn.poll(.1):
            if is_preempt_requested():
                rospy.loginfo('preempt requested')
                return None

        sentence, semantics = conn.recv()

        rospy.loginfo('Received result: {}, {}'.format(sentence, semantics))
        conn.close()

        return HMIResult(sentence, semantics)
