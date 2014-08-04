#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
animating
"""

import numpy as np
from matplotlib import pyplot as plt
from matplotlib.transforms import Affine2D
from matplotlib import animation

from paths import *
p1 = straight_path(np.r_[-5, 2], np.r_[5, 2])
p2 = concatenate_paths(straight_path(np.r_[-5, np.tan(np.pi/6)*(-5+1) + np.sin(np.pi/3)],
                                     np.r_[-1, 2*np.sin(np.pi/3)]),
                       circular_path(0, 2, 2*np.pi/3, np.pi/2),
                       straight_path(np.r_[0, 2], np.r_[5, 2]))

def animate_robots(path_abscisse_tuples, **args):
    fig = plt.figure()
    fig.set_dpi(100)
    fig.set_size_inches(7, 6.5)
    xlim, ylim = zip(*[p(s) for p, _ in path_abscisse_tuples
                            for s in (0, p.length)])
    margin = 10
    ax = plt.axes(xlim=(min(xlim)-margin, max(xlim)+margin),
                  ylim=(min(ylim)-margin, max(ylim)+margin))

    height, width = 1.75, 4.

    patches = [plt.Rectangle((-150, -150), width=width, height=height, fc=('r', 'g', 'g', 'r')[i],
                             alpha=(1,1,0.5,0.5)[i])
               for i, _ in enumerate(path_abscisse_tuples)]
    for patch in patches:
        patch.center = (-150, -150)
        ax.add_patch(patch)

    paths = [l for p, _ in path_abscisse_tuples
               for l in plot(p, decorate=True)]
    print paths

    def init():
        #for patch in patches:
        #    patch.center = (-150, -150)
        #    ax.add_patch(patch)
        return patches

    def animate(i):
        for j, el in enumerate(path_abscisse_tuples):
            path, abscisses = el
            s = abscisses[i]
            x, y = path(s)
            patches[j].set_xy((x-width/2., y-height/2.))
            theta = orientation(path, s)
            t2 = Affine2D().rotate_around(x, y, theta) + ax.transData
            patches[j].set_transform(t2)
        return patches

    anim = animation.FuncAnimation(fig, animate, 
                                   init_func=init, 
                                   frames=len(path_abscisse_tuples[0][1]), 
                                   interval=100,
                                   blit=True)
    plt.show()


