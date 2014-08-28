#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
This module provides utility functions to make an animation from a simulation.
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.transforms import Affine2D
from matplotlib import animation

import paths

def vehicle_color(vehicle_index):
    """Returns a string representing the color of a vehicle given its index in the list of 
    vehicles to be displayed.
    In this way we can distinguish vehicles coming from different paths when paths have
    common sections."""
    colors = ['r', 'g']
    return colors[vehicle_index%len(colors)]

def init_vehicle_patch(vehicle, index):
    """Return a matplotlib shape representing the vehicle.
    To avoid strange behaviour of matplotlib, the vehicle intial position is outside the
    axis limits."""
    length, width = vehicle.get_data(('length', 'width'))
    return plt.Rectangle(xy=(-150, -150), width=length, height=width,
                         fc=vehicle_color(index))

def update_vehicle_patch(vehicle_patch, vehicle, path, position, ax):
    """Updates the vehicle patch object according to the vehicle position.
    The matplotlib axis is necessary to make things work in the rotation of the object.
    This might be removed in the future."""
    x, y = path(position)
    length, width = vehicle.get_data(('length', 'width'))
    vehicle_patch.set_xy((x-length/2., y-width/2.))
    theta = paths.orientation(path, position)
    t2 = Affine2D().rotate_around(x, y, theta) + ax.transData
    vehicle_patch.set_transform(t2)
    return None

def init_figure_and_axes(xlim, ylim):
    """Set up the pyplot figure and axes and return them."""
    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(20, 8)
    margin = 10
    ax = plt.axes(xlim=(min(xlim)-margin, max(xlim)+margin),
                  ylim=(min(ylim)-margin, max(ylim)+margin))
    return fig, ax


def animate_vehicles(vehicles, save=False):
    xlim, ylim = zip(*[v.path(s) for v in vehicles
                                 for s in (0, v.path.length)])
    fig, ax = init_figure_and_axes(xlim, ylim)

    vehicles_patches = [init_vehicle_patch(v, i) for i, v in enumerate(vehicles)]
    for patch in vehicles_patches:
        ax.add_patch(patch)

    paths_patches = [patch for v in vehicles for patch in paths.plot(v.path, decorate=True)]

    def init():
        return vehicles_patches

    def animate(i):
        for ind, vehicle in enumerate(vehicles):
            path, abscisses = vehicle.path, vehicle.get_history('position')
            current_position = abscisses[i]
            update_vehicle_patch(vehicles_patches[ind], vehicle, path, current_position, ax)
        return vehicles_patches

    simulation_length = len(vehicles[0].get_history('position'))

    anim = animation.FuncAnimation(fig, animate, 
                                   init_func=init, 
                                   frames=simulation_length,
                                   interval=10,
                                   blit=True)
    if save:
        #TODO not working currently.
        anim.save('basic_animation.mp4', fps=30)
    else:
        plt.show()


