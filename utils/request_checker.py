
import rospy


class RequestVerifiers:
    """
    This class verifies and processes any incoming controller request.

    Controllers send information to a set of actuators to make the system reach
    a desired state given by a set of set points. Different controllers could
    share a common set of actuators to perform this corrective action. This
    class acts as a measure to ensure the active controllers don't conflict
    with each other in this regard. That is, this class guarantees all the
    active controllers will not have any actuators in common. When a controller
    is activated, any other controller that might conflict with the requested
    controller is deactivated.

    The active controllers are stored in the dictionary active_ctrls, where the
    keys are the controllers being considered and the values are just booleans.
    If the value is True, then the controller is active. Otherwise, it is
    inactive. The active controllers are the ones that are going to be ran in
    the control evaluator class.

    This class uses 1 ROS message and 1 ROS service to function:

        - Activate Controller Topic: This class is the publisher of this topic
          and it states what controllers are active after processing a request
          made by another part of the code.

        - Controller Change Service: This class acts as a server for this
          service. It is in charge of reading a request to activate or
          deactivate the controllers. After processing this, it will publish the
          active controllers on the topic mentioned above.

    Notice how I didn't mention the specific names for the topic and the service.
    That's because when creating an instance of this class, the user provides all
    of this information. This makes the class reusable for other systems and other
    types of controls within a system.
    """

    def __init__(self, msgs_info, ctrls_info):

        self.msgs_info = msgs_info
        self.ctrls_info = ctrls_info
        self.active_ctrls = dict.fromkeys(ctrls_info, False)  # Dictionary that indicates if a controller is active

        self._init_rospy_dependencies()

    def process_ctrl_request_handler(self, ctrl_request_srv):
        """
        The service handler for processing the incoming requests to change
        the current active controllers.

        For a set of controllers, the method can:

        - Switch controllers

        - Activate a controller

        - Deactivate controllers

        Look at the specific methods that perform these actions to get more
        information on how these work.
        """

        # If only one of the process methods was requested, then process the request
        if ctrl_request_srv.switch + ctrl_request_srv.activate + ctrl_request_srv.deactivate == 1:
            requested_ctrls = [ctrl for ctrl in self.ctrls_info if getattr(ctrl_request_srv, ctrl)]
            if ctrl_request_srv.switch:
                valid_request = self._switch_ctrls(requested_ctrls)
            elif ctrl_request_srv.activate:
                valid_request = self._activate_ctrl(requested_ctrls)
            else:
                valid_request = self._deactivate_ctrls(requested_ctrls)

            # Only update active controllers if the request was valid
            if valid_request:
                self._publish_active_controllers()
            return valid_request

        # Multiple process methods were requested or nothing was requested
        return False

    def _activate_ctrl(self, requested_ctrl):
        """
        This method will only activate one controller.

        If more controllers are requested to be activated, then the request
        will be skipped. This is to prevent any unintended effects from
        happening.

        When a controller is activated, any other conflicting controllers, i.e.
        any controller that uses some of the actuators the requested controller
        uses, will be removed in favor of the requested controller.
        """

        # Only one controller can be activated at a time
        if len(requested_ctrl) != 1:
            return False

        new_ctrl = requested_ctrl[0]
        self.active_ctrls[new_ctrl] = True
        self._check_ctrl_consistency(new_ctrl)
        return True

    def _check_ctrl_consistency(self, new_ctrl):
        """
        This method checks what controllers are currently active and if these
        conflict with the requested controller. That is, they both share some
        actuators in common to perform their respective corrective actions. If
        it does conflict, then the old active controller is switched off since
        this method assumes the requested controller has a higher priority than
        the ones currently active.
        """

        new_ctrl_actuators = self.ctrls_info[new_ctrl]["actuators"]
        for ctrl in self.ctrls_info:
            ctrl_actuators = self.ctrls_info[ctrl]["actuators"]
            actuators_in_common = ctrl_actuators.intersection(new_ctrl_actuators)
            if len(actuators_in_common) != 0 and new_ctrl != ctrl:  # If controllers have some actuators in common
                self.active_ctrls[ctrl] = False

    def _deactivate_ctrls(self, requested_ctrls):
        """
        This method will deactivate all the controllers that were requested.
        """

        for ctrl in requested_ctrls:
            self.active_ctrls[ctrl] = False
        return True

    def _init_rospy_dependencies(self):
        """
        Initialize ROS dependencies.

        The service defined here is not used in the code, but the publisher is
        used for publishing the current active controls.
        """

        self.ctrl_request_srv = rospy.Service(self.msgs_info["verifier"]["service"],
                                              self.msgs_info["verifier"]["srv"],
                                              self.process_ctrl_request_handler)

        self.active_ctrls_pub = rospy.Publisher(self.msgs_info["active_ctrls"]["topic"],
                                                self.msgs_info["active_ctrls"]["msg"],
                                                self.msgs_info["active_ctrls"]["queue_size"])

    def _publish_active_controllers(self):

        active_ctrls_msg = self.msgs_info["active_ctrls"]["msg"]()  # Creates message object, so values can be published
        for active_ctrl, active_status in self.active_ctrls.items():
            setattr(active_ctrls_msg, active_ctrl, active_status)
        self.active_ctrls_pub.publish(active_ctrls_msg)

    def _switch_ctrls(self, requested_ctrls):
        """
        This method will activate one controller, that is not currently active,
        and deactivate another controller, that is currently active.

        This method of processing the request expects two controllers and must
        be active and the other must be inactive. If this is not the case, the
        code will reject the request and not do anything.

        When a controller is activated, any other conflicting controllers, i.e.
        any controller that uses some of the actuators the requested controller
        uses, will be removed in favor of the requested controller.
        """

        # There must be two requested controllers and one of controllers must be active and the other inactive
        if len(requested_ctrls) != 2 \
                or (self.active_ctrls[requested_ctrls[0]] == self.active_ctrls[requested_ctrls[1]]):
            return False

        for ctrl in requested_ctrls:
            if not self.active_ctrls[ctrl]:
                new_ctrl = ctrl  # This will always be assigned, so no need to add it before the loop
            self.active_ctrls[ctrl] = not self.active_ctrls[ctrl]  # Flips active state of controller
        self._check_ctrl_consistency(new_ctrl)
        return True
