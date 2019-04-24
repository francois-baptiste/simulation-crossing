#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
This module defines some controllers, which can be used by the central simulator to control
the vehicles in the intersection.
"""


def bang_bang_controller(vehicles, priority):
    """This controller makes the non prioritary vehicle brake at maximum while the other one
    accelerates at maximum.
    Once a critical position is reached, both vehicles are allowed to go at maximum
    acceleration."""
    limit_position = 100
    if vehicles[priority].position < limit_position:
        return (-1, 1) if priority else (1, -1)
    else:
        return (1, 1)


def cap_command(u):
    """Returns a command capped between -1. and 1."""
    return min(max(u, -1.), 1.)


def constant_spacing_controller(vehicles, priority):
    """This controller implements the constant spacing controller defined in Rajamani."""
    u_lead = 1.
    xsi, omega = 1., 1.
    v_lead, v_follow = vehicles[priority], vehicles[1 - priority]
    constant_spacing = 10.
    u = (u_lead - 2 * xsi * omega * (v_follow.speed - v_lead.speed) -
         omega ** 2 * (v_follow.position - v_lead.position + constant_spacing))
    return (cap_command(u), u_lead) if priority else (u_lead, cap_command(u))
