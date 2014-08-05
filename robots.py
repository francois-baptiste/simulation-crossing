#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
This module manages the robots dynamics.
The `System` class stores all the states of the robots since its initialization.
A robot state is represented by a numpy array with 3 dimensions: position, speed and acceleration.
The states of the robots belonging to the `System` can be integrated given commands.
```
system = System(np.r_[0, 0, 0, 0, 0, 0], dt=0.1)
commands = np.array([np.r_[1, 0] for _ in range(10)])
system._integrate_dynamics_with_command(commands[0])
print system
system._integrate_dynamics_with_command_n_times(commands)
print system
system.reset_robot_by_index(0)
print system
```
#TODO: créer une fonction de construction du contrôleur par une chaine de caractère
ça permet de créer le contrôleur directement avec les paramètres du système.
"""
import numpy as np
from collections import namedtuple

from simple_controllers import constant_speed

RobotData = namedtuple('RobotData', 'length width max_speed min_acc max_acc')

class System(object):
    ROBOT_STATE_SIZE = 3 # position, speed and acceleration
    DEFAULT_ROBOT_DATA = RobotData(length=4., width=2., max_speed=40., min_acc=-3., max_acc=1.)

    def __init__(self,
                 initial_state,
                 robots_data=None,
                 controller=constant_speed,
                 dt=0.1):
        if len(initial_state)%System.ROBOT_STATE_SIZE:
            raise ValueError('Invalid initial state size')
        self._state_size = len(initial_state)
        self._count_robots = len(initial_state)/System.ROBOT_STATE_SIZE
        self._robots_history = np.array([np.array(initial_state)])
        self._controller = controller
        self._dt = dt # the time interval between two entries in the history
        if robots_data is None:
            robots_data = [System.DEFAULT_ROBOT_DATA for _ in xrange(self.count_robots)]
        elif len(robots_data) != self.count_robots:
            raise ValueError('Inconsistent robots data')
        self._robots_data = robots_data


    def __str__(self):
        return str(self._robots_history)

    @property
    def count_robots(self):
        return self._count_robots

    @property
    def robots_history(self):
        return self._robots_history

    @property
    def last_state(self):
        return self._robots_history[-1]

    @property
    def robots_data(self):
        return self._robots_data

    def get_robot_history_by_index(self, i):
        return self._robots_history[:, i]

    def get_robots_history_by_indexes(self, indexes=None):
        if indexes is None: indexes = xrange(self.count_robots)
        return [get_robot_history_by_index(i) for i in indexes]

    def change_controller(self, controller):
        self._controller = controller

    def reset_robot_by_index(self, i):
        s = System.ROBOT_STATE_SIZE
        self._robots_history[-1, s*i:s*(i+1)] = np.r_[-100, 0, 0]
        return self.last_state

    def _integrate_dynamics_with_command(self, command):
        """Integrates the dynamics for `dt`, stores the resulting system's state into the
           history and returns it.
           Note that `command` is only stored in the history and will be used in the
           next integration step."""
        size = System.ROBOT_STATE_SIZE
        current = self.last_state
        # integrate positions and speeds:
        new = np.r_[current[:-1] + self._dt*current[1:], 0].reshape((1, self._state_size))
        # cap the speeds:
        max_speeds = np.array([r.max_speed for r in self.robots_data])
        new[0, 1::size] = np.maximum(0,
                                     np.minimum(max_speeds,
                                                new[0, 1::size]))
        # update new commands:
        new[0, 2::size] = command
        self._robots_history = np.r_[self._robots_history, new]
        return new

    def _integrate_dynamics_with_command_n_times(self, commands):
        """Integrates the dynamics `len(commands)` times, stores the resulting states
           into the history and returns them.
           Note that the last command is only stored in the history and will be used in the
           next integration step."""
        return [self._integrate_dynamics_with_command(c) for c in commands][-1]

    def integrate_dynamics(self):
        """Integrates the dynamics for `dt`, using the internal controller, stores the
           resulting system's state into the history and returns it.""" 
        command = self._controller(self.last_state)
        return self._integrate_dynamics_with_command(command)

    def integrate_dynamics_n_times(self, n=1):
        for t in xrange(n):
            self.integrate_dynamics()
        return self.last_state
