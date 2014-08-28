#! /usr/bin/env python
# -*- coding: utf-8 -*-
# vim:fenc=utf-8
#
# Copyright © 2014 Sébastien Diemer <sebastien.diemer@mines-paristech.fr>

"""
Module defines cost function used for MPC control
"""
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from animation import animate_robots
from paths import *

p1 = straight_path(np.r_[-5, 2], np.r_[5, 2])
p2 = concatenate_paths(straight_path(np.r_[-5, np.tan(np.pi/6)*(-5+1) + np.sin(np.pi/3)],
                                     np.r_[-1, 2*np.sin(np.pi/3)]),
                       circular_path(0, 2, 2*np.pi/3, np.pi/2),
                       straight_path(np.r_[0, 2], np.r_[5, 2]))
dt = 0.2
L1, L2 = 1., 1.

def project(s):
    return p2(s)[:, 0] + 5

def psi(x, theta=1.):
    sqrt_theta = theta**0.5
    x_sup_minus_theta = x > -sqrt_theta
    x_inf_theta = x < sqrt_theta
    return (np.exp(-1./(theta-(x**2)*x_sup_minus_theta*x_inf_theta))
            - np.negative(x_sup_minus_theta)*np.exp(-1./theta)
            - np.negative(x_inf_theta)*np.exp(-1./theta))

def plateau(x, theta=1.):
    x_in_theta_0 = (-theta < x)*(x < 0)
    x_sup_0 = (x >= 0)
    return x_in_theta_0*psi(x, theta) + x_sup_0*psi(0, theta)


def cost_from_abscisses(s1, s2, priority=1, debug=False):
    margin = 0.1
    max_cost = 1e15
    cost = -s1 - s2
    L1, L2 = 1, 1
    s1_lim, s2_lim = 2.7, 4.
    #if priority == 1:
    s1_fw, s1_rr, s2_fw = s1 + L1/2, s1 - L1/2, s2 + L2/2
    p1_s2 = project(s2_fw)
    if debug: print s1_fw, s1_rr, p1_s2, s1_rr - p1_s2
    return (plateau(p1_s2 - s1_fw)*plateau(s2_fw - s2_lim)*1e10
            + cost)
    if s1_rr > p1_s2 + margin:
        return cost
    elif p1_s2 >= s2_lim:
        if debug: print "Danger"
        return max_cost
    else:
        return (s1_rr <= p1_s2 + margin)/(s2_lim - p1_s2) + cost
   # else:
   #     p1_s2 = project(s2 - L2/2)
   #     s1 = s1 + L1/2
   #     if p1_s2 > s1 + margin:
   #         return cost
   #     elif s2 >= s2_lim:
   #         return max_cost
   #     else:
   #         return (p1_s2 <= s1 + margin)/(s2_lim - s2) + cost

assert cost_from_abscisses(0, 10) > 1e5
assert cost_from_abscisses(0, 4.1) > 1e5
assert cost_from_abscisses(2, 4.1) > 1e5
assert cost_from_abscisses(10, 4.1) < 1e2

print cost_from_abscisses(0, 0)
print cost_from_abscisses(20, 20)
print cost_from_abscisses(0, 10)
print cost_from_abscisses(10, 0)

#x = np.linspace(0, 20, 200)
#y = np.linspace(0, 20, 200)
#x, y = np.meshgrid(x, y)
#print y.shape
#print project(y)
#z = np.zeros(x.shape)
#for i in range(len(x)):
#    for j in range(len(x[0])):
#        z[i, j] = cost_from_abscisses(x[i, j], y[i, j])
#z = np.array([cost_from_abscisses(a, b) for a in x for b in y])

#fig = plt.figure()
#ax = fig.gca(projection='3d')
#print x.shape, y.shape, z.shape
#p = ax.plot_surface(x, y, z, rstride=4, cstride=4, linewidth=0)
#plt.show()


def integrate(s, v, vmax, a):
    s += v*dt
    v = max(min(vmax, v + a*dt), 0)
    return s, v

S1, V1, v1max, a1min, a1max = 0, 1, 1.5, -0.8, 0.7
S2, V2, v2max, a2min, a2max = 2, 1, 1.5, -0.8, 0.7
def cost(commands, debug=False):
    c = 0
    s1, v1 = S1, V1
    s2, v2 = S2, V2
    for u1, u2 in commands.reshape(len(commands)/2, 2):
        s1, v1 = integrate(s1, v1, v1max, u1)
        s2, v2 = integrate(s2, v2, v2max, u2)
        nc = cost_from_abscisses(s1, s2, priority=1, debug=debug)
        c += nc
        if debug: print 'cost: ', nc
    return c

from scipy.optimize import minimize
from scipy.optimize import fmin_tnc
Amax = np.zeros(40)
Amax[::2] = a1max
Amax[1::2] = a2max
Amin = np.zeros(40)
Amin[::2] = a1min
Amin[1::2] = a2min
x0 = -np.ones(40)
x = fmin_tnc(cost, x0, approx_grad=True, bounds=zip(Amin, Amax))
print x
print 'optimal: ', cost(x[0])
print 'initial: ', cost(x0)
u = np.array([-1, 1]*20)
u = np.array([1, -1]*20)
u = x[0]

u1 = u[::2]
u2 = u[1::2]
#u1 = np.ones(20)
#u2 = -np.ones(20)

s1, v1 = [S1], [V1]
s2, v2 = [S2], [V2]
for i in xrange(len(u1)):
    s, v = integrate(s1[-1], v1[-1], v1max, u1[i])
    s1.append(s)
    v1.append(v)
    s, v = integrate(s2[-1], v2[-1], v2max, u2[i])
    s2.append(s)
    v2.append(v)

animate_robots([(p1, s1), (p2, s2)])

#print s1

traj1 = p1(np.array(s1))
traj2 = p2(np.array(s2))

plt.plot(traj1[:,0], traj1[:,1])
plt.plot(traj2[:,0], traj2[:,1])
plt.figure()
plt.plot(u1)
plt.plot(u2)
plt.plot(s1, project(np.array(s2)))
plt.show()

