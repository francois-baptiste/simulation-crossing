#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
Un module pour décrire les véhicules
"""
import numpy as np
from matplotlib import pyplot as plt

DT = 0.01

DEFAULT_DATA = {
    'L': 3., # Vehicle length
    'B': 1.5, # Forward semi-length
    'C': 1.5, # L-B
    'H': 0.6, # The height of the center of gravity
    'H_aero': 1., # the height at which the equivalent aerodynamic force act
    'M': 1626, # The weight of the vehicle
    'J': 15., # moment of inertia
    'A': 2.08216, # 1.6 + 0.00056*(M-765)
    'R': 0.3, # The radius of the tire
    'g': 9.81,
    'rho_air': 1.23,
    'Cd': 0.29, # aerodynamic drag coefficient
    'Cr': 0.01, # rolling drag
    'Cf': .1, # rotation axis friction
    'lag_throttle': 0.2,
    'lag_brake': 0.2,
    'brake_max_pressure': 150, # 150 bar the pressure applied when full brake
    'alpha': 0.01,
    'Kc': 1.,
    'Kb_forward': 13.33,
    'Kb_rear': 6.666,
    'eta_f': 2.82,
    'eta_1': 3.56,
    'eta_2': 2.19,
    'eta_3': 1.41,
    'eta_4': 1.,
    'eta_5': 0.83,
    'shift_map_up': ((15, 58), (30, 90), (45, 120), (60, 150)),
    'shift_map_down': ((7, 45), (17, 63), (32, 90), (48, 125)),
   }

def convert_to_brake_and_throttle(command):
    brake = command if command < 0 else 0.
    throttle = command if command > 0 else 0.
    return brake, throttle

def gear_map(a, b, throttle):
    return (a if throttle <= 0.3
            else b if throttle >= 0.8
            else (b-a)/0.5*(throttle-0.3) + a)


def lag_filter_update_value(lag, old_value, new_input):
    lag_coeff = DT/lag
    return (1-lag_coeff)*old_value + lag_coeff*new_input

class Vehicle():
    def __init__(self, init_state=(0, 0, 0, 4, 0, 30, 0), data=DEFAULT_DATA):
        self._vehicle_data = data
        self._states = [init_state]

    @property
    def vehicle_data(self):
        return self._vehicle_data

    @property
    def states(self):
        return self._states

    def update_state(self, command, brake, throttle, gear, position, speed, acceleration):
        self.states.append((command, brake, throttle, gear, position, speed, acceleration))

    def get_history(self, attribute):
        attributes = 'command brake throttle gear position speed acceleration'.split()
        return np.array(self.states)[:, attributes.index(attribute)]

    @property
    def command(self):
        return self.states[-1][0]

    @property
    def brake(self):
        return self.states[-1][1]

    @property
    def throttle(self):
        return self.states[-1][2]

    @property
    def gear(self):
        return self.states[-1][3]

    @property
    def position(self):
        return self.states[-1][4]

    @property
    def speed(self):
        return self.states[-1][5]

    @property
    def acceleration(self):
        return self.states[-1][6]

    @property
    def wheel_speed(self):
        R, = self.get_data(['R'])
        return self.speed/R

    @property
    def brake_pressure(self):
        return self.brake*150

    def get_data(self, requested_data_elements):
        return [self.vehicle_data[p] for p in requested_data_elements]

    def apply_command(self, command):
        lag_brake, lag_throttle = self.get_data(['lag_brake', 'lag_throttle'])
        brake_command, throttle_command = convert_to_brake_and_throttle(command)
        brake = lag_filter_update_value(lag_brake, self.brake, brake_command)
        throttle = lag_filter_update_value(lag_throttle, self.throttle, throttle_command)
        gear = self.change_gear()
        acceleration = self.get_current_acceleration()
        speed = self.speed + DT*acceleration
        position = self.position + DT*speed
        self.update_state(command=command,
                          brake=brake,
                          throttle=throttle,
                          gear=gear,
                          position=position,
                          speed=speed,
                          acceleration=acceleration)

    def change_gear(self):
        map_up, map_down = self.get_data(['shift_map_up', 'shift_map_down'])
        if self.gear < 5:
            a, b = map_up[self.gear - 1]
            if self.wheel_speed > gear_map(a, b, self.throttle):
                return self.gear + 1
        if self.gear > 1:
            a, b = map_down[self.gear - 2]
            if self.wheel_speed < gear_map(a, b, self.throttle):
                return self.gear - 1
        return self.gear

    def max_torque(self):
        gear_ratio, drive_ratio = self.get_data(['eta_%s' % self.gear, 'eta_f'])
        wheel_speed_RPM = 60/(2*np.pi)*self.wheel_speed
        engine_speed_RPM = gear_ratio*drive_ratio*wheel_speed_RPM
        return 528.7 + 0.152*engine_speed_RPM - 0.0000217*engine_speed_RPM**2

    def get_current_drive_force(self):
        gear_ratio, drive_ratio, R = self.get_data(['eta_%s' % self.gear, 'eta_f',
                                                      'R'])
        wheel_torque = self.throttle*gear_ratio*drive_ratio*self.max_torque()
        return wheel_torque/R*0.7

    def get_current_brake_force(self):
        K_fw, K_rr, alpha, R = self.get_data(['Kb_forward', 'Kb_rear', 'alpha',
                                                'R'])
        brake_torque = self.brake_pressure*(K_fw + K_rr)*min(1., self.wheel_speed/alpha)
        return brake_torque/R
                
    def get_current_drag_force(self):
        rho_air, A, Cd = self.get_data(['rho_air', 'A', 'Cd'])
        return -0.5*np.sign(self.speed)*rho_air*A*Cd*self.speed**2

    def get_current_rolling_resistance_force(self):
        Cr, M, g = self.get_data(['Cr', 'M', 'g'])
        normal_force = -M*g
        return normal_force*Cr*min(1., abs(self.speed))*np.sign(self.speed)

    def get_current_acceleration(self):
        M, = self.get_data(['M'])
        return sum((self.get_current_drive_force(),
                    self.get_current_brake_force(),
                    self.get_current_drag_force(),
                    self.get_current_rolling_resistance_force()))/M

