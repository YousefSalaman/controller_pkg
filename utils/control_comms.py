
import rospy

from .request_checker import RequestVerifiers
from .control_evaluator import ControllerEvaluators


class ControllerCommunications:
    """
    The purpose of this class is to establish communication between the
    control systems and other parts of the code. One can instance this
    class binds different sections of the code, the control systems and a
    set of actuators. This works for any set of arbitrary actuators as long
    as the information below is provided to the class when creating an
    instance.

    Here's the information the class needs by attribute (note that both of
    these are dictionaries):

    - msgs_info:

      {
        "actuators": -> Information relating to the actuator topic with a dictionary as value shown below
          {
          "attributes": -> Dictionary with the names of all the actuators as keys and their default value as values
            {
            ...
            "actuator_i": -> Name of the ith actuator as the key and as a value its default value
            ...
            }
          "msg": -> ROS Message object corresponding to the actuators
          "queue_size": -> Queue size for the actuator publisher
          "topic": -> Topic name for the set of actuators
          }
        "measurements": -> Information relating to the measurement topic with a dictionary as value shown below
          {
          "attributes": -> Names of all the measurements in the message
          "msg":  -> ROS Message object corresponding to the measurements
          "topic":  -> Topic name for the measurements
          }
        "set_points": -> Information relating to the set point topic with a dictionary as value shown below
          {
          "attributes": -> Names of all the set points in the message
          "msg":  -> ROS Message object corresponding to the set points
          "topic":  -> Topic name for the set points
          }
      }

    - ctrls_info:

        {
        ...
        ctrl_i: -> Name of the ith controller in the control system. Each of these comes with a dictionary with info as
                   values as shown below
          {
          "group_key": -> Group key for the controller
          "inputs" -> Names of the inputs to the controller
          "runner": -> Runner object for the controller "ctrl_i"
          }
        ...
        }
        """

    def __init__(self, node_name, msgs_info, ctrls_info):

        rospy.init_node(node_name)

        self.req_checker = RequestVerifiers(msgs_info, ctrls_info)
        self.ctrl_evaluator = ControllerEvaluators(msgs_info, ctrls_info)

        rospy.spin()
