#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
Simulator for crossing scenario
"""

import numpy as np
from scipy.optimize import minimize

from deterministic import Robot, determine_priority
from mpc_controller import build_MPC_controller

TIME_HORIZON = 75
HORIZON = 10
dt = 0.2
STATE_SIZE = 6

K = 1.
D_safe = 5.
ROBOT1 = Robot(length=4., pos=0., speed=30., max_speed=30., acc=1., amin=-5., amax=1.5)
ROBOT2 = Robot(length=4., pos=2., speed=25., max_speed=30., acc=1., amin=-3., amax=0.5)
VMAX = 30.
AMAX = (ROBOT1.amax, ROBOT2.amax)
AMIN = (ROBOT1.amin, ROBOT2.amin)
MAX_JERK = 10.
PRIORITY = determine_priority(ROBOT1, ROBOT2)
print 'Priority: ', PRIORITY

x0 = (np.array([(r.pos, r.speed, r.acc) for r in (ROBOT1, ROBOT2)])).flatten()
print x0

from robots import *
def simulator(robot1, robot2, time_horizon):
    states = np.zeros((time_horizon, STATE_SIZE))
    states[0] = np.r_[robot1.pos, robot1.speed, robot1.acc, robot2.pos, robot2.speed, robot2.acc]
    system = System(np.r_[0, 39, 0, 0, 39, 0], dt=0.1)
    #ctrl = build_bang_bang_controller(0)
    ctrl = build_MPC_controller(HORIZON, PRIORITY, system.robots_data, paths)
    #ctrl = build_constant_speed_controller()
    system.change_controller(ctrl)
    system.integrate_dynamics_n_times(time_horizon)
    return system.robots_history
    for t in xrange(1, time_horizon):
        states[t] = simulate(states[t-1], ctrl)
    return states


from animation import animate_robots
from paths import *

paths = build_reference_path(30.)
p1, p2 = paths
D = p1.length*0.5

result = simulator(ROBOT1, ROBOT2, TIME_HORIZON)

plt.plot(result[:,2], label='a1')
plt.plot(result[:,5], label='a2')
plt.plot(result[:,0], label='s1')
plt.plot(result[:,3], label='s2')
plt.plot(result[:,1], label='v1')
plt.plot(result[:,4], label='v2')
plt.legend()
plt.show()

#init = np.r_[x0, PRIORITY, TIME_HORIZON]
#p_prime = (np.r_[x0, integrate_full_brake(*init)]).reshape((TIME_HORIZON+1, STATE_SIZE))
animate_robots([(p2, result[:,3]),
                (p1, result[:,0])])
#                (p1, p_prime[:,0]),
#                (p2, p_prime[:,3])])
