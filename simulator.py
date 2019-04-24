#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
This module simulates the evolution of two vehicles, using a constant spacing controller.
"""
import controllers

def simulate(vehicles, time_horizon):
    priority = 0
    controller = lambda v: controllers.constant_spacing_controller(v, priority)
    for t in range(1, time_horizon):
        u1, u2 = controller(vehicles)
        for i, u in enumerate((u1, u2)):
            vehicles[i].apply_command(u)
    return vehicles
