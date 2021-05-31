#!/usr/bin/env python
import rospy

from pyrunner.runners import executors


class ControllerEvaluators(object):
    """This class gathers all the necessary information, runs the controllers
    and it publishes the actuator values.

    This class uses 4 different ROS topics to function:

        - Set Point Topic: The class is subscribed to this topic to gather
          the different set points of the system and use them to guide the
          control system.

        - Measurement Topic: The class is subscribed to this topic to
          gather the necessary sensor measurements as feedback for the
          control system.

        - Active Controller Topic: The class is subscribed to this topic
          to verify and only run the controllers requested by another
          part of the code.

        - Actuator Topic: The class publishes to this topic to indicate
          what should be the values of each actuator.

    Notice how I didn't mention the specific names for the topics above. That's
    because when creating an instance of this class, the user provides all of
    this information. This makes the class reusable for other systems and other
    types of controls within a system.
    """

    def __init__(self, msgs_info, ctrls_info):

        self.msgs_info = msgs_info  # Save relevant information about the messages
        self.ctrls_info = ctrls_info  # Save relevant information about the control systems

        # Define other attributes
        self._init_rospy_dependencies()
        self._create_information_storages()

    def active_ctrl_callback(self, active_ctrls_msg):
        """Callback to update the active controllers"""

        for ctrl in self.active_ctrls:
            self.active_ctrls[ctrl] = getattr(active_ctrls_msg, ctrl)

    def measurement_callback(self, measurements_msg):
        """Callback to update the measurements taken from the sensor.

        Everytime this is ran, aside from internally updating the values
        taken from the different sensors, the active controllers are ran
        to calculate what new values the actuators should take to reach
        the values given by the set points. After this, the values for
        the actuators are also published to its respective topic.
        """

        for measure in self.measurements:
            self.measurements[measure] = getattr(measurements_msg, measure)

        self._publish_actuator_values(self.run_controllers())

    def run_controllers(self):
        """Run the active controllers.

        The results of the control system's computations are stored in
        a dictionary whose keys are the name of the relevant actuators.

        Running the controllers is delegated to an object that contains
        enough information about the control system to run it without
        any need of knowing the structure and initial conditions of the
        control system in question. These objects are called "runners"
        since they run the active controllers.

        A thing to note is the names of the relevant inputs for the
        controllers and the names attributes in the set points and
        measurement messages must match for proper code execution. If
        this doesn't happen, then this may result in a fatal error.

        Another thing to note is that the code assumes the set points
        and measurements are names differently since the controller
        needs to register the set points and the measurements as
        separate variables.

        The last thing to note is the callback won't actually run the
        controllers unless the values have been given to the set points
        and measurements. This is to avoid any numerical errors in the
        calculations within the controllers.
        """

        ctrl_outputs = {}
        for ctrl, is_active in self.active_ctrls.items():
            if is_active and self._is_ctrl_initialized(ctrl):
                ctrl_inputs = self._extract_controller_inputs(ctrl)
                ctrl_outputs.update(executors.run(ctrl, ctrl_inputs))

        return ctrl_outputs

    def set_point_callback(self, set_points_msg):
        """Callback to update the set points of the control systems."""

        for set_point in self.set_points:
            self.set_points[set_point] = getattr(set_points_msg, set_point)

    def _is_ctrl_initialized(self, ctrl):
        """Check if set points and measurements have numeric values for a
        given controller.

        Basically, this just verifies if the arguments for the controller
        have been entered or not. Controllers that have all their inputs set
        will be stored and this is used to indicate if the values for the
        actuators should be taken from its default value or the controller.
        """

        if ctrl not in self.initialized_ctrls:
            measures_defined = all(self.measurements[measure] is not None for measure in self.measurements
                                   if measure in self.ctrls_info[ctrl]["inputs"])
            set_points_defined = all(self.set_points[set_point] is not None for set_point in self.set_points
                                     if set_point in self.ctrls_info[ctrl]["inputs"])
            if set_points_defined and measures_defined:
                self.initialized_ctrls.add(ctrl)

        return ctrl in self.initialized_ctrls

    def _create_information_storages(self):
        """Create dictionaries to store relevant information.

        The class uses these attributes, which are gathered in the different
        callbacks, to run the controllers.
        """

        self.initialized_ctrls = set()  # Initialized controllers

        # Define the attributes "active_ctrls", "set_points", "measurements" with their respective default values
        attr_iter = zip(["active_ctrls", "set_points", "measurements"], [False, None, None])
        for attr_name, default in attr_iter:
            msg_attrs = self._get_message_attributes(attr_name)
            setattr(self, attr_name, dict.fromkeys(msg_attrs, default))

    def _extract_controller_inputs(self, ctrl):

        # Extract the set points that are in the controller's inputs
        ctrl_inputs = {set_point: value for set_point, value in self.set_points.items()
                       if set_point in self.ctrls_info[ctrl]["inputs"]}

        # Extract the measurements in the controller's inputs and add them to the inputs
        ctrl_inputs.update({measure: value for measure, value in self.measurements.items()
                            if measure in self.ctrls_info[ctrl]["inputs"]})

        return ctrl_inputs

    def _get_message_attributes(self, attr_name):

        msg_cls = self.msgs_info[attr_name]["msg"]  # Get the respective message class

        # The __slots__ dunder attribute contains the names of the attributes the instances must have
        # and _slot_types contains the types of the attributes in the same order they are stored in
        # __slots__
        msg_attr_iter = zip(msg_cls.__slots__, msg_cls._slot_types)  # Get attribute names and types

        return [msg_attr_name for msg_attr_name, msg_attr_type in msg_attr_iter
                if msg_attr_type != "Header"]

    def _init_rospy_dependencies(self):
        """Initialize ROS dependencies.

        The attributes are defined here, so there is a hard reference of
        the publishers/subscribers somewhere in the code, but these are
        not used in the code per se. However, the publisher is used to
        send back the values to the actuators of the system.
        """

        self.actuator_pub = rospy.Publisher(self.msgs_info["actuators"]["topic"],
                                            self.msgs_info["actuators"]["msg"],
                                            queue_size=self.msgs_info["actuators"]["queue_size"])

        self.set_points_sub = rospy.Subscriber(self.msgs_info["set_points"]["topic"],
                                               self.msgs_info["set_points"]["msg"],
                                               self.set_point_callback)

        self.measurement_sub = rospy.Subscriber(self.msgs_info["measurements"]["topic"],
                                                self.msgs_info["measurements"]["msg"],
                                                self.measurement_callback)

        self.active_ctrls_sub = rospy.Subscriber(self.msgs_info["active_ctrls"]["topic"],
                                                 self.msgs_info["active_ctrls"]["msg"],
                                                 self.active_ctrl_callback)

    def _publish_actuator_values(self, ctrl_outputs):
        """Publish controller results to relevant actuator message.

        This takes the outputs of all the active controllers and
        passes the resulting values to the message. For this to work,
        the values of the controllers must have keys that have the
        same name as the actuators in the actuator message.

        However, if the an actuator value is not in any of the
        controller outputs, then the default value defined by the user
        is used instead to prevent any mishaps.
        """

        actuators_msg = self.msgs_info["actuators"]["msg"]()  # Creates a message object, so we can publish the values
        for actuator, default_value in self.msgs_info["actuators"]["defaults"].items():
            if actuator in ctrl_outputs:
                value = ctrl_outputs[actuator]
            else:
                value = default_value  # This is the actuator's default value
            setattr(actuators_msg, actuator, value)  # State the values the actuators should have in the message

        self.actuator_pub.publish(actuators_msg)  # Publish values to topic after finishing
