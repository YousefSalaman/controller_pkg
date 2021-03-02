

from .srv import *
from ...utils import *
from .msg import ActivateMovementCtrls, MovementMeasures

NODE_NAME = ""

MSGS_INFO = {}

CTRLS_INFO = {}

if __name__ == "__main__":

    control_comms.ControllerCommunications(NODE_NAME, MSGS_INFO, CTRLS_INFO)
