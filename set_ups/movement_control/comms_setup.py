
from ...utils import *
from .srv import MovementCtrlChange
from .msg import ActivateMovementCtrls, MovementMeasures, MovementSetPoints, MotorValues

NODE_NAME = ""  # Node name for the movement controllers

RUNNER = None  # Runner for the movement controllers

_MOTORS = ["motor_" + str(i) for i in range(1, 9)]  # List of motor names

# Information about the ROS messages/services related to the movement controls
MSGS_INFO = {
    "active_ctrls":
        {
            "attributes": {"forward", "right", "yaw", "pitch", "roll", "depth"},
            "msg": ActivateMovementCtrls,
            "queue_size": 3,
            "topic": "act_mov_ctrls"
        },
    "actuators":
        {
            "attributes": dict.fromkeys(_MOTORS, 1500),
            "msg": MotorValues,
            "queue_size": 3,
            "topic": "mov_motor_values"
        },
    "measurements":
        {
            "attributes":
                {
                    "z", "yaw", "pitch", "roll", "qx", "qy", "qz", "qw",
                    "vel_x", "vel_y", "vel_z"
                },
            "msg": MovementMeasures,
            "topic": "mov_measures"
        },
    "set_points":
        {
            "attributes":
                {
                    "set_forward", "set_right",
                    "set_x", "set_y", "set_z", "set_yaw", "set_pitch", "set_roll"
                },
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

    control_comms.ControllerCommunications(NODE_NAME, RUNNER, MSGS_INFO, CTRLS_INFO)
