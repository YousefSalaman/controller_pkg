#!/usr/bin/env python

from __future__ import print_function

import time
import numpy as np
from matplotlib import pyplot

import pyrunner.runners as runners
import pyrunner.runners.seq_runner as seq_runner


def DIP_LQR():
    """Test LQR controller for DIP system."""

    # Initialize gain
    gain = np.loadtxt('DIPSGain.out')
    state, set_state = yield

    while True:  # Controller gain calculation loop
        u = set_state - np.dot(gain, state)
        if u > 5:
            u = 5
        if u < -5:
            u = -5
        state, set_state = yield {'u': u}


DIP_LQR_exec = seq_runner.Executor("DIP_LQR", DIP_LQR(), ['state', 'set_state'])


def DIP_sys():
    """A double inverted pendulum's (DIP's) dynamics."""

    # System's initial conditions
    pos_state = np.array([.05, .01, .01])
    vel_state = np.array([.03, 0, 0])
    u = yield np.append(pos_state, vel_state)

    while True:  # Main system loop

        a0, a1, a2 = pos_state
        a3, a4, a5 = vel_state
        t0 = a1 + a2
        t1 = np.sin(t0)
        t2 = np.sin(a2)
        t3 = np.sin(a1)
        t4 = np.cos(t0)
        t5 = np.cos(a1)
        t6 = np.cos(a2)
        t7 = 2*a1
        t8 = 2*a2
        t9 = np.cos(t7 - t8)
        t10 = np.cos(t7 + a2)
        t11 = np.cos(t7 + t8)
        t12 = np.cos(t7)
        t13 = np.cos(t8)

        # Acceleration components of the degrees of freedom (dofs)
        c0 = (((3.168848501e-6*t4 + 5.272537842e-6*t5)*(0.0046064424*t6 + 0.0065257934) - 3.168848501e-6*(0.0092128848*t6 + 0.0139580366)*t4)*(4.905*t1 - 0.0762*t2*a4**2) - (-2.414662557762e-7*t4*t6 + 5.29395592033938e-23*t4 + 5.691704600439e-7*t5)*(0.29651706*t1 + 0.49336452*t3 + 0.0092128848*t2*a4*a5 + 0.0046064424*t2*a5**2) + (0.00035101091088*t2**2 + 2.16840434497101e-19*t6 + 0.00045129974256)*(0.000180693695*u + 3.168848501e-6*t1*a4**2 + 6.337697002e-6*t1*a4*a5 + 3.168848501e-6*t1*a5**2 + 5.272537842e-6*t3*a4**2 - 0.00080972686249*a3))/(-2.19719850209398e-25*t9 - 3.30872245021211e-24*t10 + 1.83997286901463e-10*t11 - 8.24044992051559e-9*t12 + 9.92616735063633e-24*t6 - 1.08558399271864e-8*t13 + 4.63278883091187e-8)
        c1 = (-(-0.0023032212*t4*t6 + 0.0054290214*t5)*(0.000180693695*u + 3.168848501e-6*t1*a4**2 + 6.337697002e-6*t1*a4*a5 + 3.168848501e-6*t1*a5**2 + 5.272537842e-6*t3*a4**2 - 0.00080972686249*a3) - (4.905*t1 - 0.0762*t2*a4**2)*(-7.9683864406146e-8*t10 - 4.7890807395613e-8*t11 + 3.64614046222062e-7*t6 + 5.81531232661015e-7) - (1.5844242505e-6*t4**2 - 1.0411930789e-5)*(0.29651706*t1 + 0.49336452*t3 + 0.0092128848*t2*a4*a5 + 0.0046064424*t2*a5**2))/(-2.19719850209398e-25*t9 - 3.30872245021211e-24*t10 + 1.83997286901463e-10*t11 - 8.24044992051559e-9*t12 + 9.92616735063633e-24*t6 - 1.08558399271864e-8*t13 + 4.63278883091187e-8)
        c2 = (((0.030226*t4 + 0.050292*t5)*(0.0762*t6 + 0.10795) - 0.5*(0.0092128848*t6 + 0.0139580366)*t4)*(0.000180693695*u + 3.168848501e-6*t1*a4**2 + 6.337697002e-6*t1*a4*a5 + 3.168848501e-6*t1*a5**2 + 5.272537842e-6*t3*a4**2 - 0.00080972686249*a3) + (4.905*t1 - 0.0762*t2*a4**2)*(-1.59367728812292e-7*t10 - 4.7890807395613e-8*t11 - 1.32583236574932e-7*t12 + 7.29228092444124e-7*t6 + 1.16579840651143e-6) - (-0.5*(3.168848501e-6*t4 + 5.272537842e-6*t5)*t4 + 7.349598204e-6*t6 + 1.0411930789e-5)*(0.29651706*t1 + 0.49336452*t3 + 0.0092128848*t2*a4*a5 + 0.0046064424*t2*a5**2))/(-2.19719850209398e-25*t9 - 3.30872245021211e-24*t10 + 1.83997286901463e-10*t11 - 8.24044992051559e-9*t12 + 9.92616735063633e-24*t6 - 1.08558399271864e-8*t13 + 4.63278883091187e-8)
        accel = np.array([c0, c1, c2], dtype='float64')

        # Get current components of the DIP's state vector
        vel_state += .01*accel  # Velocity components of the dofs
        pos_state += .01*vel_state  # Position components of the dofs
        u = yield np.append(pos_state, vel_state)


sys_gen = DIP_sys()  # Generator to extract the current state of the DIP system


if __name__ == "__main__":

    # Test simulation initializations
    dt = .01
    sim_time = 5
    _set_state = 0
    start_time = time.time()
    sample_size = int((1 / dt) * sim_time)
    state_matrix = np.zeros((6, sample_size))
    time_vec = np.arange(0, sim_time, dt)

    # Run test system simulation
    _state = next(sys_gen)
    for i in range(sample_size):
        state_matrix[:, i] = _state
        _u = runners.executors.run('DIP_LQR', {'state': _state, 'set_state': _set_state})['u']
        _state = sys_gen.send(_u)

    end_time = time.time()
    print(end_time-start_time)

    # Plot resulting state
    plot_nums = 6
    pyplot.figure()
    for i in range(plot_nums):
        subplot = pyplot.subplot(plot_nums, 1, i + 1)
        pyplot.plot(time_vec, state_matrix[i, :])
