#!/usr/bin/env python

from __future__ import print_function


class MockCtrlRequest(object):
    """Mock controller request for testing the RequestVerifiers class."""

    __slots__ = ('switch', 'activate', 'deactivate',  # Request change types
                 'ctrl_1', 'ctrl_2', 'ctrl_3', 'ctrl_4')  # Possible controllers to request

    def __init__(self):

        self.reset()

    def set(self, *attrs):
        """Set indicated request values to True"""

        for attr in attrs:
            setattr(self, attr, True)

    def reset(self):
        """Set request values to False"""

        for attr in self.__slots__:
            setattr(self, attr, False)


class MockRequestVerifiers(object):
    """ A mock class verifies and processes any incoming controller request.

    This mock class directly shadows the RequestVerifiers class in the ros_comms
    subpackage, but it removes the rospy dependencies to test out the controller
    switching mechanisms.
    """

    def __init__(self, ctrls_info):

        self.ctrls_info = ctrls_info
        self.active_ctrls = dict.fromkeys(ctrls_info, False)  # Dictionary that indicates if a controller is active

    def process_ctrl_request(self, ctrl_request):

        # If only one of the process methods was requested, then process the request
        valid_request = False
        if ctrl_request.switch + ctrl_request.activate + ctrl_request.deactivate == 1:
            requested_ctrls = [ctrl for ctrl in self.ctrls_info if getattr(ctrl_request, ctrl)]
            if ctrl_request.switch:
                valid_request = self._switch_ctrls(requested_ctrls)
            elif ctrl_request.activate:
                valid_request = self._activate_ctrl(requested_ctrls)
            else:
                valid_request = self._deactivate_ctrls(requested_ctrls)

        return valid_request, self.active_ctrls

    def reset(self):
        """Deactivates all the controllers."""

        self.active_ctrls = dict.fromkeys(self.active_ctrls, False)

    def _activate_ctrl(self, requested_ctrl):

        # Only one controller can be activated at a time
        if len(requested_ctrl) != 1:
            return False

        new_ctrl = requested_ctrl[0]
        self.active_ctrls[new_ctrl] = True
        self._check_ctrl_consistency(new_ctrl)
        return True

    def _check_ctrl_consistency(self, new_ctrl):

        new_ctrl_actuators = self.ctrls_info[new_ctrl]["actuators"]
        for ctrl in self.ctrls_info:
            ctrl_actuators = self.ctrls_info[ctrl]["actuators"]
            actuators_in_common = ctrl_actuators.intersection(new_ctrl_actuators)
            if len(actuators_in_common) != 0 and new_ctrl != ctrl:  # If controllers have some actuators in common
                self.active_ctrls[ctrl] = False

    def _deactivate_ctrls(self, requested_ctrls):

        for ctrl in requested_ctrls:
            self.active_ctrls[ctrl] = False
        return True

    def _switch_ctrls(self, requested_ctrls):

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


# This test system will have a set of 4 actuators and 4 controllers

# The controller pairs (1,3) and (2,4) form a partition on the set of actuators, respectively (i.e. they don't use
# the same actuators, so they don't interfere with each other)

ctrls_info = {  # Info about the actuators of each controller
    'ctrl_1': {'actuators': {'act_1', 'act_2'}},
    'ctrl_2': {'actuators': {'act_3', 'act_4'}},
    'ctrl_3': {'actuators': {'act_1', 'act_3'}},
    'ctrl_4': {'actuators': {'act_2', 'act_4'}},
}

verifier = MockRequestVerifiers(ctrls_info)  # Create verifier for our mock system

ctrl_req = MockCtrlRequest()  # Create controller request


def make_request(*attrs):
    """Helper function to make requests"""

    ctrl_req.set(*attrs)
    resp = verifier.process_ctrl_request(ctrl_req)
    ctrl_req.reset()

    return resp


if __name__ == '__main__':

    # If one requests more than request change type at the same time, this will result in a failed request
    print(make_request('activate', 'switch'))

    # Activating controllers

    # This is a request to activate ctrl_1
    # It should result in (True, {ctrl_1: True, ctrl_2: False, ctrl_3: False, ctrl_4: False})
    print(make_request('ctrl_1', 'activate'))

    # This is a request to activate ctrl_2
    # It should result in (True, {ctrl_1: True, ctrl_2: True, ctrl_3: False, ctrl_4: False})
    print(make_request('ctrl_2', 'activate'))

    # The following will activate controller 3, but it will deactivate the other currently active controllers since
    # these have some actuators in common and the system will assume the new controller should take priority over the
    # old ones. This is done to avoid any potential conflict that could happen if two or more uncompensated controllers
    # that use the same actuators may bring (i.e. this could lead to undefined behavior if the controllers were
    # designed separately.)
    print(make_request('ctrl_3', 'activate'))

    verifier.reset()  # This all the controllers to False

    # The following is a failed request since it tries to activate 2 controllers at once, when only one controller per
    # activation request can be done
    # It should result in (True, {ctrl_1: False, ctrl_2: False, ctrl_3: False, ctrl_4: False})
    print(make_request('ctrl_1', 'ctrl_2', 'activate'))

    # Similarly, if no controllers are put in the request then the request will fail
    print(make_request('activate'))

    # Deactivating controllers

    # Activate a couple of controllers for the sake of testing this behavior
    # It should result in {ctrl_1: True, ctrl_2: True, ctrl_3: False, ctrl_4: False}
    make_request('ctrl_1', 'activate')
    print(make_request('ctrl_2', 'activate')[1])

    # This deactivates both ctrl_1 and ctrl_2
    # # It should result in (True, {ctrl_1: False, ctrl_2: False, ctrl_3: False, ctrl_4: False})
    print(make_request('ctrl_1', 'ctrl_2', 'deactivate'))

    # Switching controllers

    # Activate a controller for the sake of testing this behavior
    # It should result in {ctrl_1: True, ctrl_2: False, ctrl_3: False, ctrl_4: False}
    print(make_request('ctrl_1', 'activate'))

    # It should result in {ctrl_1: False, ctrl_2: True, ctrl_3: False, ctrl_4: False}
    print(make_request('ctrl_1', 'ctrl_2', 'switch'))

    # Switching components will also turn off any conflicting components
    # In here, I will activate ctrl_1, so both ctrl_1 and ctrl_2 are active. Now, I will switch ctrl_1 for ctrl_3, which
    # will turn off ctrl_1 off, but it will also turn off ctrl_2 since this conflicts with ctrl_3, which will result in
    # {ctrl_1: False, ctrl_2: False, ctrl_3: True, ctrl_4: False}
    print(make_request('ctrl_1', 'activate'))  # Results in {ctrl_1: True, ctrl_2: True, ctrl_3: False, ctrl_4: False}
    print(make_request('ctrl_1', 'ctrl_3', 'switch'))

    # The following will result in a failed requests
    # There all going to result in (False, {ctrl_1: False, ctrl_2: False, ctrl_3: True, ctrl_4: False})
    # To switch components, one component needs to be active and while the other must be inactive
    print(make_request('ctrl_1', 'ctrl_2', 'switch'))

    # Only two controllers can be requested for switching
    print(make_request('ctrl_1', 'switch'))
    print(make_request('ctrl_1', 'ctrl_2', 'ctrl_3', 'switch'))
