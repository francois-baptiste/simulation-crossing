#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""

"""

def controller(vehicles, priority):
    if vehicles[priority].position < 100:
        return (-1, 1) if priority else (1, -1)
    else:
        return (1, 1)

def build_linear_controller(vehicles, priority):
    d_0 = vehicles[priority].position - vehicles[1-priority].position
    s_lim, d_safe = 140, 10
    K = .1 # gain
    slope = (d_0 - d_safe)/(vehicles[priority].position - s_lim)
    def linear_controller(vehicles):
        if vehicles[priority].position - vehicles[1-priority].position < d_safe:
            u2 = min(max(-K*(vehicles[1-priority].speed-(1-slope)*vehicles[priority].speed),
                         -1), 1)
            return (u2, 1) if priority else (1, u2)
        return 1, 1
    return linear_controller

def cap_command(u):
    return min(max(u, -1.), 1.)

def constant_spacing_controller(vehicles, priority):
    u_lead = 1.
    xsi, omega = 1., 1.
    v_lead, v_follow = vehicles[priority], vehicles[1-priority]
    constant_spacing = 10.
    u = (u_lead - 2*xsi*omega*(v_follow.speed - v_lead.speed) -
         omega**2*(v_follow.position - v_lead.position + constant_spacing))
    return (cap_command(u), u_lead) if priority else (u_lead, cap_command(u))


def simulator(vehicles, time_horizon):
    priority = 0
    controller = lambda v: constant_spacing_controller(v, priority)
    for t in xrange(1, time_horizon):
        u1, u2 = controller(vehicles)
        for i, u in enumerate((u1, u2)):
            vehicles[i].apply_command(u)


from animation import animate_robots
from paths import *
from vehicle import Vehicle

paths = build_reference_path(30.)
p1, p2 = paths
v1 = Vehicle(init_state=(0, 0, 0, 4, np.random.rand()*10, np.random.rand()*20+20, 0))
v2 = Vehicle(init_state=(0, 0, 0, 4, np.random.rand()*10, np.random.rand()*20+20, 0))
v1 = Vehicle(init_state=(0, 0, 0, 4, 0, 0, 0))
v2 = Vehicle(init_state=(0, 0, 0, 4, 20, 30, 0))
simulator([v1, v2], 1000)
animate_robots([(p1, v1.get_history('position')),
                (p2, v2.get_history('position'))])
plt.plot(v1.get_history('position'), label='v1pos')
plt.plot(v2.get_history('position'), label='v2pos')
plt.plot(v1.get_history('speed'), label='v1speed')
plt.plot(v2.get_history('speed'), label='v2speed')
plt.plot(v1.get_history('acceleration'), label='v1acc')
plt.plot(v2.get_history('acceleration'), label='v2acc')
plt.legend()
plt.show()
