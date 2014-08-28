#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
A simple demo of the simulator.
"""

import numpy as np
from matplotlib import pyplot as plt

import paths, simulator
from animation import animate_vehicles
from vehicle import Vehicle

def simple_demo():
    ref_paths = paths.build_reference_path(30.)
    p1, p2 = ref_paths
    v1 = Vehicle(p1, init_state=(0, 0, 0, 4, np.random.rand()*10, np.random.rand()*20+20, 0))
    v2 = Vehicle(p2, init_state=(0, 0, 0, 4, np.random.rand()*10, np.random.rand()*20+20, 0))
    v1 = Vehicle(p1, init_state=(0, 0, 0, 4, 0, 0, 0))
    v2 = Vehicle(p2, init_state=(0, 0, 0, 4, 20, 30, 0))
    simulator.simulate([v1, v2], 1000)

    # show the simulation with an animation
    animate_vehicles([v1, v2], save=False)

    # display the simulation data on a graph
    plt.figure()
    plt.plot(v1.get_history('position'), label='v1pos')
    plt.plot(v2.get_history('position'), label='v2pos')
    plt.plot(v1.get_history('speed'), label='v1speed')
    plt.plot(v2.get_history('speed'), label='v2speed')
    plt.plot(v1.get_history('acceleration'), label='v1acc')
    plt.plot(v2.get_history('acceleration'), label='v2acc')
    plt.plot(v1.get_history('position') - v2.get_history('position'), label='delta')
    plt.legend()
    plt.show()

if __name__ == '__main__':
    simple_demo()
