

from ...utils import *
from .srv import MovementCtrlChange
from .msg import ActivateMovementCtrls, MovementMeasures, MovementSetPoints, ThrusterValues

NODE_NAME = ""  # Node name for the movement controllers

RUNNER = None  # Runner for the movement controllers

# Information about the ROS messages/services related to the movement controls
MSGS_INFO = {
    "active_ctrls":
        {
            "attributes": {"forward", "backward", "left", "right", "yaw", "pitch", "depth"},
            "msg": ActivateMovementCtrls,
            "queue_size": 3,
            "topic": "act_mov_ctrls"
        },
    "actuators":
        {
            "attributes": {"thruster_" + str(i): 1500 for i in range(1, 9)},
            "msg": ThrusterValues,
            "queue_size": 3,
            "topic": "thruster_values"
        },
    "measurements":
        {
            "attributes": {"yaw", "pitch", "depth"},
            "msg": MovementMeasures,
            "topic": "mov_measures"
        },
    "set_points":
        {
            "attributes": {"set_forward", "set_backward", "set_left", "set_right", "set_yaw", "set_pitch", "set_depth"},
            "msg": MovementSetPoints,
            "topic": "mov_set_points"
        },
    "verifier":
        {
            "service": "mov_ctrl_req",
            "srv": MovementCtrlChange
        }
}

# TODO: Verify again what thrusters are assigned to each controller/movement
_ACTUATORS = ["thruster_" + str(i) for i in range(1, 9)]
_CONFIG_1 = set(_ACTUATORS[:4])  # Thrusters 1 to 4
_CONFIG_2 = set(_ACTUATORS[4:])  # Thrusters 5 to 8

# Information about the movement controllers
CTRLS_INFO = {
    "backward":
        {
            "actuators": _CONFIG_1,
            "inputs": ["set_backward"]
        },
    "depth":
        {
            "actuators": _CONFIG_2,
            "inputs": ["depth", "set_depth"]
        },
    "forward":
        {
            "actuators": _CONFIG_1,
            "inputs": ["set_forward"]
        },
    "left":
        {
            "actuators": _CONFIG_1,
            "inputs": ["set_left"]
        },
    "pitch":
        {
            "actuators": _CONFIG_2,
            "inputs": ["pitch", "set_pitch"]
        },
    "right":
        {
            "actuators": _CONFIG_1,
            "inputs": ["set_right"]
        },
    "yaw":
        {
            "actuators": _CONFIG_1,
            "inputs": ["yaw", "set_yaw"]
        }
}

if __name__ == "__main__":

    control_comms.ControllerCommunications(NODE_NAME, RUNNER, MSGS_INFO, CTRLS_INFO)
