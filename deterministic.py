#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
Le but de ce module est de faire une simulation d'intersection déterministe
impliquant deux véhicules qui empruntent deux routes qui se rejoignent.
"""
import numpy as np
from matplotlib import pyplot as plt

from collections import namedtuple

def minimimum_positive_root(A, B, C):
    """Returns the minimum positive root of a second order polynom A x^2 + B x + C"""
    if A == 0:
        t = -C/B
        if t > 0: return t
        else: raise ValueError('No positive root')
    Delta = B**2 - 4*A*C
    if Delta >= 0:
        solutions = (s for s in ((-B - Delta**0.5)/(2*A), (-B + Delta**0.5)/(2*A)) if s > 0)
        if solutions:
            return min(solutions)
    raise ValueError('No positive root')

def minimum_time_to_straight_line(a, b, a1_min, v1_0, s1_0, a2_max, v2_0, v2_max, s2_0):
    """Straight line has equation y = a.x + b"""
    A, B, C = (a2_max - a*a1_min)*0.5, v2_0 - a*v1_0, s2_0 - a*s1_0 - b
    t = minimimum_positive_root(A, B, C)
    args = (s1_0, v1_0, a1_min, s2_0, v2_0, a2_max)

    if a2_max == 0 and a1_min == 0:
        return [(t, args)]
    elif a1_min == 0:
        time_to_v2_max = (v2_max - v2_0)/a2_max
        time_to_v1_min = 0.
    elif a2_max == 0:
        time_to_v1_min = -v1_0/a1_min
        time_to_v2_max = 0.
    else:
        time_to_v2_max = (v2_max - v2_0)/a2_max
        time_to_v1_min = -v1_0/a1_min

    if t < min(time for time in (time_to_v2_max, time_to_v1_min) if time > 0):
        return [(t, args)]
    elif a2_max == 0 or 0 < time_to_v1_min <= time_to_v2_max:
        t = time_to_v1_min
        s1_0, v1_0, a1_min = integrate_abscisse(s1_0, v1_0, a1_min)(t), 0., 0.
        s2_0, v2_0 = integrate_abscisse(s2_0, v2_0, a2_max)(t), integrate_speed(v2_0, a2_max)(t)
        if v2_0 == v2_max: a2_max = 0.
    elif a1_min == 0 or 0 < time_to_v2_max < time_to_v1_min:
        t = time_to_v2_max
        s1_0, v1_0 = integrate_abscisse(s1_0, v1_0, a1_min)(t), integrate_speed(v1_0, a1_min)(t)
        s2_0, v2_0, a2_max = integrate_abscisse(s2_0, v2_0, a2_max)(t), v2_max, 0.
    else:
        raise ValueError('Uncaught case')
    return ([(t, args)] +
            minimum_time_to_straight_line(a, b, a1_min, v1_0, s1_0, a2_max, v2_0, v2_max, s2_0))

def integrate_speed(v0, a):
    return lambda t: v0 + a*t

def integrate_abscisse(s0, v0, a):
    return lambda t: 0.5*a*t**2 + v0*t + s0

def minimum_time_to_escape(a, b1, b2, robot1, robot2):
    """intercept b1 for the straight line for which robot1 has priority
       intercept b2 for the straight line for which robot2 has priority"""
    max_speed = 2.
    result1 = minimum_time_to_straight_line(1./a, -b1/a,
                                            robot2.amin, robot2.speed, robot2.pos,
                                            robot1.amax, robot1.speed, max_speed, robot1.pos)
    plot_coordination_diagram(result1, reverse_axis=True, color='b')
    t1 = sum(t for t, _ in result1)
    result2 = minimum_time_to_straight_line(a, b2,
                                            robot1.amin, robot1.speed, robot1.pos,
                                            robot2.amax, robot2.speed, max_speed, robot2.pos)
    plot_coordination_diagram(result2, reverse_axis=False, color='g')
    t2 = sum(t for t, _ in result2)
    print((t1, 1), (t2, 2))
    return min((t1, 1), (t2, 2))

def determine_priority(robot1, robot2):
    """This function makes the assumption that robot1's path and robot2's path are correctly
       parameterized in the sense that the curvilinear abscisses are the same for the shared
       portion of the path. This implies that the obstacle region is delimited by the two
       straight lines y = x + L and y = x - L, where L = L1/2 + L2/2 is the minimum space
       between the centers of the two vehicles.
       The function returns 0 if robot1 has priority, else 1."""
    L = 0.5*(robot1.length + robot2.length)
    # first case: robot 1 speeds up and robot 2 brakes:
    result1 = minimum_time_to_straight_line(1., L,
                                            robot2.amin, robot2.speed, robot2.pos,
                                            robot1.amax, robot1.speed, robot1.max_speed,
                                            robot1.pos)
    t1 = sum(t for t, _ in result1)
    # second case: robot 2 speeds up and robot 1 brakes:
    result2 = minimum_time_to_straight_line(1., L,
                                            robot1.amin, robot1.speed, robot1.pos,
                                            robot2.amax, robot2.speed, robot2.max_speed,
                                            robot2.pos)
    t2 = sum(t for t, _ in result2)
    return min((t1, 0), (t2, 1))[1]

def plot_coordination_diagram(time_calculation_result, reverse_axis=False, color='b'):
    dt = 0.01
    times, args = list(zip(*time_calculation_result))
    t = [np.arange(0, tf + dt, dt) for tf in times]
    if not reverse_axis:
        s1 = [integrate_abscisse(*arg[:3])(time) for time, arg in zip(t, args)]
        s2 = [integrate_abscisse(*arg[3:])(time) for time, arg in zip(t, args)]
    else:
        s1 = [integrate_abscisse(*arg[3:])(time) for time, arg in zip(t, args)]
        s2 = [integrate_abscisse(*arg[:3])(time) for time, arg in zip(t, args)]
    for gs1, gs2 in zip(s1, s2):
        plt.plot(gs1, gs2, '%s-' % color)


Robot = namedtuple('Robot', 'length pos speed max_speed acc amin amax')
robot1 = Robot(length=3., pos=0, speed=1.2, max_speed=2., acc=0., amin=-0.5, amax=0.8)
robot2 = Robot(length=3., pos=0, speed=1.2, max_speed=2., acc=0., amin=-0.2, amax=0.3)
print(minimum_time_to_escape(2., -2, 3, robot1, robot2))


#s1_0, v1_0, a1 = 0, 1.2, -0.2
#s2_0, v2_0, a2 = 0, 1.8, 0.8
#a, b = 2., 2.
#result = minimum_time_to_straight_line(a, b, a1, v1_0, s1_0, a2, v2_0, 2, s2_0)
#minimum_time = sum((t for t, _ in result))
#r = np.linspace(0, 5, 10)
#plt.plot(r, a*r - 2)
#plt.plot(r, a*r + 3)
#plt.axis('equal')
#plt.show()
