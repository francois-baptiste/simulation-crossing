#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
A module defining basic controllers
"""
import numpy as np

def build_bang_bang_controller(priority, robots_data):
    def bang_bang_controller(state):
        return np.array([r.min_acc*(i!=priority) + r.max_acc*(i==priority)
                         for i, r in enumerate(robots_data)])
    return bang_bang_controller

def build_constant_speed_controller():
    def constant_speed_controller(state):
        return np.r_[0, 0]
    return constant_speed_controller

constant_speed = build_constant_speed_controller()

def build_full_brake_controller(robots_data):
    def full_brake(state):
        return np.array([r.min_acc for r in robots_data])
    return full_brake
