#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
This module implements a MPC controller for a system of two robots.
The principle of the controller is the following: at each timestep, a finite horizon optimization
is computed for a given cost functions, under some constraints.
The MPC controller returns the command for the first timestep from this optimal solution.
The following controller uses a cost function which combines two objectives: 
    * bring the robots out of the intersection
    * limitate violent changes of the commands (for smoother driving experience)
Three types of constraints are implemented:
    * the dynamics constraints linking the position, speed and acceleration between two timesteps
    * the bounds on acceleration
    * the safety constraint
The safety constraint is the most complicated to handle. More on that later #TODO
The optimization problem is solved using scipy SLSQP solver.
"""

import numpy as np
from scipy.optimize import minimize

from robots import System
from simple_controllers import build_full_brake_controller, build_bang_bang_controller

STATE_SIZE = System.ROBOT_STATE_SIZE*2
print STATE_SIZE
dt = 0.1

def indexes_of_same_variables(first_indexes, horizon, state_size=STATE_SIZE):
    """Since we construct states for a finite time horizon, we deal with arrays consisting of
    the same variable at different time steps. Given a collection of indexes for the first time
    step, this utility function returns the indexes of all the future values of the variables
    identified by these indexes."""
    return np.array([np.array(first_indexes) + i*state_size for i in xrange(horizon)]).flatten()

def MPC_cost(states_until_horizon, *init):
    x = np.r_[init, states_until_horizon]
    return (sum((x[8::STATE_SIZE] - 50)**2) + sum((x[11::STATE_SIZE] - 50)**2) +
            sum((x[8::STATE_SIZE] - x[2:-STATE_SIZE:STATE_SIZE])**2) +
            sum((x[11::STATE_SIZE] - x[5:-STATE_SIZE:STATE_SIZE])**2))

def MPC_eq_constraints(states_until_horizon, robots_data, *init):
    horizon = (len(states_until_horizon))/STATE_SIZE
    x = np.r_[init , states_until_horizon]
    # only indexes 0, 1, 3, 4 are subject to equality constraint
    indexes = indexes_of_same_variables((0, 1, 3, 4), horizon)
    commands = zip(x[2::STATE_SIZE], x[5::STATE_SIZE])
    system = System(init, robots_data)
    system._integrate_dynamics_with_command_n_times(commands)
    evolution = system.robots_history.flatten()[indexes]
    result = (x[STATE_SIZE:] - x[0:-STATE_SIZE] - dt*x[1:-STATE_SIZE+1])[indexes]
    return result

def build_MPC_ineq_constraints(robots_data, paths):
    def MPC_ineq_constraints(states_until_horizon, priority):
        """If priority == 0 then the abscisse of the first vehicle must remain greater than the 
           abscisse of the other vehicule. Else if priority == 1 roles are inverted.
           More precisely we can define delta as `s1 - s2 - D_safe` if robot 1 is prioritary (else
           `s2 - s1 - D_safe`) and we must ensure that delta is positive when the prioritary robot
           enters the critical zone.
           This must be ensured as soon as the robots enter the critical zone, but not necessarily
           before.
           To cope with this, we define a limit linear evolution for delta with a slope ensuring
           that delta is at zero when the robots enter the collision zone."""
        x = states_until_horizon
        if not priority in (0, 1): raise ValueError('Wrong priority')
        i1, i2 = (0, 3) if priority == 0 else (3, 0)
        s1 = x[i1::STATE_SIZE] # the abscisse of the prioritary vehicle
        # the approximated limit of the beginning of the collision zone
        s1_lim = 0.5*(paths[priority].length - robots_data[priority].length) 
        D_safe = sum([r.length for r in robots_data]) # twice the sum of the semi-length
        delta = (x[i1::STATE_SIZE] - x[i2::STATE_SIZE] - D_safe)
        result = delta - np.minimum(-delta[0]/(s1_lim - s1[0])*(s1 - s1[0]) + delta[0], 0)
        return result
    return MPC_ineq_constraints

def MPC_bounds(robots_data, horizon):
    bounds = [[(None, None), (0, r.max_speed), (r.min_acc, r.max_acc)]
              for r in robots_data]*(1 + horizon) # initial_state + horizon
    bounds = reduce(lambda x,y: x+y, bounds)
    return bounds

def build_MPC_controller(horizon, priority, robots_data, paths):
    bounds = MPC_bounds(robots_data, horizon)
    MPC_ineq_constraints = build_MPC_ineq_constraints(robots_data, paths)
    bang_bang = build_bang_bang_controller(priority, robots_data)
    def MPC_controller(state):
        x0 = tuple(state)
        system = System(initial_state=x0,
                        robots_data=robots_data,
                        controller=build_full_brake_controller(robots_data))
        system.integrate_dynamics_n_times(n=horizon)
        first_guess = system.robots_history.flatten()
        opt = minimize(MPC_cost, 
                       first_guess,
                       args=x0,
                       method='SLSQP',
                       constraints=(
                                    dict(type='eq', fun=MPC_eq_constraints, args=(robots_data,)+x0),
                                    dict(type='ineq', fun=MPC_ineq_constraints, args=(priority,)),
                                   ),
                       bounds=bounds) 
        if opt.success:
            u1, u2 = opt.x[2:6:3] # corresponds to the command of the first time step
            #print MPC_ineq_constraints(opt.x, priority)
        else:
            u1, u2 = bang_bang(state)
            print 'Optimization failed'
            print opt
        return np.r_[u1, u2]
    return MPC_controller
