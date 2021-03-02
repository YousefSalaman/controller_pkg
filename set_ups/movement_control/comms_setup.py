

from ...utils import *
from .srv import MovementCtrlChange
from .msg import ActivateMovementCtrls, MovementMeasures, MovementSetPoints, ThrusterValues

NODE_NAME = ""

MSGS_INFO = {}

CTRLS_INFO = {}

if __name__ == "__main__":

    control_comms.ControllerCommunications(NODE_NAME, MSGS_INFO, CTRLS_INFO)
