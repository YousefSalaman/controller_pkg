

import os
import sys

# Add paths of the respective packages, so Python can find them
control_pkg_path = ""  # Path to directory that contains the controller_pkg directory
mov_control_path = os.path.join(control_pkg_path, "controller_pkg", "src")
sys.path.extend([control_pkg_path, mov_control_path])

from movement_control import *  # Package with the msg, srv, and other ros stuff
from controller_pkg.utils import ControllerCo  # Controller comms are stored in here


NODE_NAME = "test_node"  # Node name for the movement controllers

RUNNER = None  # Runner for the movement controllers

_MOTORS = ["motor_" + str(i) for i in range(1, 9)]  # List of motor names

# Information about the ROS messages/services related to the movement controls
MSGS_INFO = {
    "active_ctrls":
        {
            "msg": ActiveMovementCtrls,
            "queue_size": 3,
            "topic": "act_mov_ctrls"
        },
    "actuators":
        {
            "defaults": dict.fromkeys(_MOTORS, 1500),
            "msg": MotorValues,
            "queue_size": 3,
            "topic": "mov_motor_values"
        },
    "measurements":
        {
            "msg": MovementMeasures,
            "topic": "mov_measures"
        },
    "set_points":
        {
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
_CONFIG_1 = set(_MOTORS[:4])  # Motors 1 to 4
_CONFIG_2 = set(_MOTORS[4:])  # Motors 5 to 8

# Information about the movement controllers
CTRLS_INFO = {
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
    "roll":
        {
            "actuators": _CONFIG_2,
            "inputs": ["roll", "set_roll"]
        },
    "yaw":
        {
            "actuators": _CONFIG_1,
            "inputs": ["yaw", "set_yaw"]
        }
}


if __name__ == "__main__":

    ControllerCommunications(NODE_NAME, RUNNER, MSGS_INFO, CTRLS_INFO)
