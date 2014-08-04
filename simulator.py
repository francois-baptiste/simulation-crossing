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

TIME_HORIZON = 50
HORIZON = 20
dt = 0.2
STATE_SIZE = 6

K = 1.
D_safe = 3.
VMAX = 30.
AMAX = (0.5, 0.5)
AMIN = (-1., -1.)
MAX_JERK = 10.
PRIORITY = 1

x0 = np.r_[0, 5, 1, -3, 5, 1]    

def separate_epsilon_from_state(states_until_horizon):
    """We use a delta variable epsilon to loosen the safety constraint.
       Since it does not depend on time, it is appended at the end of the collection of states
       up to a certain horizon.
       This function simply removes it to have only the time dependent variables of the state."""
    return states_until_horizon, None
    return states_until_horizon[:-1], states_until_horizon[-1]

def indexes_of_same_variables(first_indexes, horizon, state_size=STATE_SIZE):
    """Since we construct states for a finite time horizon, we deal with arrays consisting of
    the same variable at different time steps. Given a collection of indexes for the first time
    step, this utility function returns the indexes of all the future values of the variables
    identified by these indexes."""
    return np.array([np.array(first_indexes) + i*state_size for i in xrange(horizon)]).flatten()

def MPC_cost(states_until_horizon, *init):
    x, epsilon = separate_epsilon_from_state(states_until_horizon)
    x = np.r_[init, x]
    return sum((x[8::STATE_SIZE] - 50)**2) + sum((x[11::STATE_SIZE] - 50)**2)
    return (sum((x[8::STATE_SIZE] - x[2:-STATE_SIZE:STATE_SIZE])**2) +
            sum((x[11::STATE_SIZE] - x[5:-STATE_SIZE:STATE_SIZE])**2) +
            TIME_HORIZON*(x[-6] - 60)**2 + TIME_HORIZON*(x[-3] - 60)**2)

def MPC_eq_constraints(states_until_horizon, *init):
    horizon = (len(states_until_horizon)-1)/STATE_SIZE
    x = np.r_[init , separate_epsilon_from_state(states_until_horizon)[0]]
    # only indexes 0, 1, 3, 4 are subject to equality constraint
    indexes = indexes_of_same_variables((0, 1, 3, 4), horizon)
    return (x[STATE_SIZE:] - x[0:-STATE_SIZE] - dt*x[1:-STATE_SIZE+1])[indexes]

def MPC_ineq_constraints(states_until_horizon, priority):
    x, epsilon = separate_epsilon_from_state(states_until_horizon)
    # if priority == 0 then the abscisse of the first vehicle must remain greater than the 
    # abscisse of the other vehicule. Else if priority == 1 roles are inverted.
    if not priority in (0, 1): raise ValueError('Wrong priority')
    i1, i2 = (0, 3) if priority == 0 else (3, 0)
    s1, s1_lim = x[i1::STATE_SIZE], D - 3
    delta = (x[i1::STATE_SIZE] - x[i2::STATE_SIZE] - D_safe)
    result = delta - np.minimum(-delta[0]/(s1_lim - s1[0])*(s1 - s1[0]) + delta[0], 0)
    return result
    return x[i1::STATE_SIZE] - x[i2::STATE_SIZE] - D_safe
    return x[i1::STATE_SIZE] - x[i2::STATE_SIZE] - D_safe + epsilon

def MPC_bounds(horizon):
    # bounds for the time dependent state variables:
    bounds = [[(None, None), (0, VMAX), (amin, amax)] for amin, amax in zip(AMIN, AMAX)]*horizon
    bounds = reduce(lambda x,y: x+y, bounds)
    # bound for epsilon
    #bounds.append((0, None))
    return bounds

def integrate_full_brake(s1_0, v1_0, a1_0, s2_0, v2_0, a2_0, priority, horizon=HORIZON):
    result = np.zeros(STATE_SIZE*horizon)
    result = np.r_[s1_0, v1_0, a1_0, s2_0, v2_0, a2_0, result]
    for t in STATE_SIZE*np.arange(horizon):
        result[STATE_SIZE+t+0] = result[t+0] + dt*result[t+1]
        result[STATE_SIZE+t+1] = max(0, min(VMAX, result[t+1] + dt*result[t+2]))
        result[STATE_SIZE+t+2] = AMAX[0] if priority == 0 else AMIN[0]
        result[STATE_SIZE+t+3] = result[t+3] + dt*result[t+4]
        result[STATE_SIZE+t+4] = max(0, min(VMAX, result[t+4] + dt*result[t+5]))
        result[STATE_SIZE+t+5] = AMIN[1] if priority == 0 else AMAX[1]
    return result[STATE_SIZE:]

def build_MPC_controller(horizon, priority):
    bounds = MPC_bounds(horizon)
    def MPC_controller(state):
        x0 = tuple(state)
        first_guess = integrate_full_brake(*x0, priority=priority, horizon=horizon)
        #first_guess = np.r_[first_guess, 0] # append epsilon
        opt = minimize(MPC_cost, 
                       first_guess,
                       args=x0,
                       method='SLSQP',
                       constraints=(
                                    dict(type='eq', fun=MPC_eq_constraints, args=x0),
                                    dict(type='ineq', fun=MPC_ineq_constraints, args=(priority,)),
                                   ),
                       bounds=bounds) 
        if opt.success:
            u1, u2 = opt.x[2:6:3] # corresponds to the command of the first time step
            print MPC_ineq_constraints(opt.x, priority)
        else:
            u1, u2 = AMIN
            print 'Optimization failed'
        return np.r_[u1, u2]
    return MPC_controller

def integrator(state, command):
    """This function integrates for one time step, a state whose components are in this order the
       position, the speed and the acceleration."""
    # we want to integrate only position and speed (integration not relevant for acceleration)
    # acceleration values are set to the new commands
    pos, speed, acc = state
    return np.r_[
                 pos + dt*speed,
                 max(0, min(speed + dt*acc, VMAX)),
                 command
                ]

def build_bang_bang_controller(priority):
    def bang_bang_controller(state):
        return np.r_[AMAX[0]*(1 - priority) + AMIN[0]*priority,
                     AMAX[1]*priority + AMIN[1]*(1 - priority)]
    return bang_bang_controller

def build_ACC_controller():
    return None

def simulate(state, controller):
    u, v = controller(state)
    return np.r_[integrator(state[:3], u),
                 integrator(state[3:], v)]

#ctrl = build_bang_bang_controller(0)
ctrl = build_MPC_controller(10, PRIORITY)
def simulator(initial_state, time_horizon):
    size = len(initial_state)
    states = np.zeros((time_horizon, size))
    states[0] = initial_state
    for t in xrange(1, time_horizon):
        states[t] = simulate(states[t-1], ctrl)
    return states


from animation import animate_robots
from paths import *

D = 30
end = D
y_straight = 2
R = 50
angle_circle = np.pi/6
assert D > angle_circle*R
angle_start = np.pi/2 + angle_circle
center = np.r_[0, y_straight - R]
x_start_circle = center[0] + np.cos(angle_start)*R
y_start_circle = center[1] + R*np.sin(angle_start)
start = center[0] - D
D_prime = D - angle_circle*R
start_slanting = np.r_[x_start_circle - np.cos(angle_circle)*D_prime,
                       y_start_circle - np.sin(angle_circle)*D_prime]
p1 = straight_path(np.r_[start, y_straight], np.r_[end, y_straight])
p2 = concatenate_paths(straight_path(start_slanting,
                                     np.r_[x_start_circle, y_start_circle]),
                       circular_path(center, R, angle_start, np.pi/2),
                       straight_path(np.r_[center[0], y_straight], np.r_[end, y_straight]))
print p1.length, p2.length
assert np.abs(p1.length - p2.length) < 1.e-6


result = simulator(x0, TIME_HORIZON)

plt.plot(result[:,2], label='a1')
plt.plot(result[:,5], label='a2')
plt.plot(result[:,0], label='s1')
plt.plot(result[:,3], label='s2')
plt.plot(result[:,1], label='v1')
plt.plot(result[:,4], label='v2')
plt.legend()
plt.show()

init = np.r_[x0, PRIORITY, TIME_HORIZON]
p_prime = (np.r_[x0, integrate_full_brake(*init)]).reshape((TIME_HORIZON+1, STATE_SIZE))
animate_robots([(p2, result[:,3]),
                (p1, result[:,0]),
                (p1, p_prime[:,0]),
                (p2, p_prime[:,3])])
