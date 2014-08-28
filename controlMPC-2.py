#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
Another try with MPC
"""

import numpy as np
from matplotlib import pyplot as plt
from scipy.optimize import minimize

from animation import animate_robots
from paths import *

start = -5
p1 = straight_path(np.r_[start, 2], np.r_[5, 2])
p2 = concatenate_paths(straight_path(np.r_[start, np.tan(np.pi/6)*(start+1) + np.sin(np.pi/3)],
                                     np.r_[-1, 2*np.sin(np.pi/3)]),
                       circular_path(0, 2, 2*np.pi/3, np.pi/2),
                       straight_path(np.r_[0, 2], np.r_[5, 2]))

TIME_HORIZON = 100
HORIZON = 20
dt = 0.2
STATE_SIZE = 6

K = 1.
D_safe = 0.3
VMAX = 30.
A1MIN = -10.
A1MAX = 1.
A2MIN = -1.5
A2MAX = 1.
MAX_JERK = 10.

def cost(x, s1_0, v1_0, a1_0, s2_0, v2_0, a2_0):
    x = np.r_[s1_0, v1_0, a1_0, s2_0, v2_0, a2_0, x]
    return (sum((x[8::STATE_SIZE] - x[2:-STATE_SIZE:STATE_SIZE])**2) +
            sum((x[11::STATE_SIZE] - x[5:-STATE_SIZE:STATE_SIZE])**2))

def constraint(x, s1_0, v1_0, a1_0, s2_0, v2_0, a2_0):
    x = np.r_[s1_0, v1_0, a1_0, s2_0, v2_0, a2_0, x]
    return np.r_[
                 x[STATE_SIZE::STATE_SIZE] - x[0:-STATE_SIZE:STATE_SIZE] - dt*x[1:-STATE_SIZE:STATE_SIZE], # s1_t+1 = s1_t + dt*v1_t
                 x[STATE_SIZE+3::STATE_SIZE] - x[3:-STATE_SIZE:STATE_SIZE] - dt*x[4:-STATE_SIZE:STATE_SIZE], # s2_t+1 = s2_t + dt*v2_t
                 x[STATE_SIZE+1::STATE_SIZE] - x[1:-STATE_SIZE:STATE_SIZE] - dt*x[2:-STATE_SIZE:STATE_SIZE], # v1_t+1 = v1_t + dt*u1_t
                 x[STATE_SIZE+4::STATE_SIZE] - x[4:-STATE_SIZE:STATE_SIZE] - dt*x[5:-STATE_SIZE:STATE_SIZE], # v2_t+1 = v2_t + dt*u2_t
                 x[STATE_SIZE+2::STATE_SIZE] - x[STATE_SIZE+5::STATE_SIZE] - (K*np.abs(D_safe - (x[0:-STATE_SIZE:STATE_SIZE] - x[3:-STATE_SIZE:STATE_SIZE]))),
                ]

def constraint_test(x):
    return np.r_[
                 x[0::STATE_SIZE] - x[3::STATE_SIZE] - D_safe
                ]

def bounds():
    return [(None, None), (0, VMAX), (A1MIN, A1MAX),
            (None, None), (0, VMAX), (A2MIN, A2MAX)]*HORIZON

def integrate_full_brake(s1_0, v1_0, a1_0, s2_0, v2_0, a2_0, horizon=HORIZON):
    result = np.zeros(STATE_SIZE*horizon)
    result = np.r_[s1_0, v1_0, a1_0, s2_0, v2_0, a2_0, result]
    for t in STATE_SIZE*np.arange(horizon):
        result[STATE_SIZE+t+0] = result[t+0] + dt*result[t+1]
        result[STATE_SIZE+t+1] = max(0, min(VMAX, result[t+1] + dt*result[t+2]))
        result[STATE_SIZE+t+2] = A1MAX
        result[STATE_SIZE+t+3] = result[t+3] + dt*result[t+4]
        result[STATE_SIZE+t+4] = max(0, min(VMAX, result[t+4] + dt*result[t+5]))
        result[STATE_SIZE+t+5] = A2MIN
    return result[STATE_SIZE:]
s1_lim, s2_lim = 2.7, 4.
def MPC(init, horizon=TIME_HORIZON):
    result = np.r_[init, np.zeros(horizon*STATE_SIZE)]
    u1, u2 = init[2:6:3]
    for t in STATE_SIZE*np.arange(horizon):
        if result[t] - result[t+3] < D_safe:
            x0 = tuple(result[t:t+STATE_SIZE])
            first_guess = integrate_full_brake(*init)
            print first_guess.shape
            u1, u2 = minimize(cost, 
                              first_guess,
                              args=x0,
                              method='SLSQP',
                              constraints=(
                                           dict(type='eq', fun=constraint, args=x0),
                                           #dict(type='ineq', fun=constraint_test),
                                          ),
                              bounds=bounds()).x[2:6:3]
        else: 
            u1 = min(A1MAX, u1 + MAX_JERK*dt)
            u2 = min(A2MAX, u2 + MAX_JERK*dt)
        result[STATE_SIZE+t+0] = result[t+0] + dt*result[t+1]
        result[STATE_SIZE+t+1] = max(0, min(VMAX, result[t+1] + dt*result[t+2]))
        result[STATE_SIZE+t+2] = u1
        result[STATE_SIZE+t+3] = result[t+3] + dt*result[t+4]
        result[STATE_SIZE+t+4] = max(0, min(VMAX, result[t+4] + dt*result[t+5]))
        result[STATE_SIZE+t+5] = u2
    return result

init = (1, 1, 0.5*A1MAX, 0, 2, 0.5*A2MAX)
x0 = integrate_full_brake(*init)
#result = minimize(cost, x0, args=init, 
#                            method='SLSQP',
#                            constraints=dict(type='eq', fun=constraint, args=init),
#                            bounds=bounds())
result = MPC(init)


plt.plot(result[2::STATE_SIZE], label='a1')
plt.plot(result[5::STATE_SIZE], label='a2')
plt.plot(result[0::STATE_SIZE], label='s1')
plt.plot(result[3::STATE_SIZE], label='s2')
plt.plot(result[1::STATE_SIZE], label='v1')
plt.plot(result[4::STATE_SIZE], label='v2')
plt.legend()
plt.show()
animate_robots([(p1, result[0::STATE_SIZE]),
                (p2, result[3::STATE_SIZE])])
