import numpy as np
from matplotlib import pyplot as plt
import matplotlib.transforms as transforms

def straight_path(start, end):
    start, end = np.array(start), np.array(end)
    v = end - start
    length = np.linalg.norm(v)
    def path(s):
        result = start + np.c_[v[0]*s, v[1]*s]/length
        if np.isscalar(s):
            return result.flatten()
        return result
    path.length = length
    return path

def circular_path(center, radius, theta_start, theta_end):
    center = np.array(center)
    def path(s):
        R = np.sign(theta_end - theta_start)*radius
        result = center + radius*np.c_[np.cos(theta_start + s/R), 
                                       np.sin(theta_start + s/R)]
        if np.isscalar(s):
            return result.flatten()
        return result
    path.length = radius*abs(theta_end - theta_start)
    return path

def concatenate_paths(*paths):
    if len(paths) == 0: return None
    if len(paths) == 1: return paths[0]
    if len(paths) == 2:
        path1, path2 = paths
        def path(s):
            if np.isscalar(s):
                return path1(s) if s <= path1.length else path2(s - path1.length)
            s1 = np.less_equal(s, path1.length)
            s2 = np.negative(s1)
            return np.r_[path1(s[s1]), path2(s[s2] - path1.length)]
        path.length = path1.length + path2.length
        return path
    else:
        path = concatenate_paths(paths[0], paths[1])
        return concatenate_paths(path, *paths[2:])

def orientation(path, s, ds=0.1):
    """Returns the angle of the path `path` at abscisse `s`."""
    dx, dy = path(s) - path(s-ds)
    return np.arctan2(dy, dx)

def build_reference_path(speed_limit, amin=3.):
    """Returns two crossing paths:
       * one straight path (P1, P2)
       * and one concatenation of a straight path (P3, P4), a curve of center C, radius R
         and angle Theta, and a last straight path (P5, P2) coinciding
         with the end of the first one (P5 is the center of (P1, P2)).
       The two paths share the same curvilinear abscisse on their common part."""
    P5 = (0, 0) # coordinates of the intersection point
    R = 50 # radius of the curved path
    Theta = np.pi/6 # angle of the curved path
    minimum_stopping_distance = speed_limit**2/(2*amin)
    assert minimum_stopping_distance > Theta*R

    # Path 1:
    P1 = (P5[0] - minimum_stopping_distance, P5[1])
    P2 = (P5[0] + minimum_stopping_distance, P5[1])
    path1 = straight_path(P1, P2)

    # Path 2:
    # - third subpath:
    subpath3 = straight_path(P5, P2)

    # - second subpath:
    C = (P5[0], P5[1] - R)
    Theta_end = np.pi/2.
    Theta_start = Theta_end + Theta
    subpath2 = circular_path(C, R, Theta_start, Theta_end)

    # - first subpath:
    d = minimum_stopping_distance - Theta*R # the length of the subpath
    P4 = subpath2(0)
    P3 = (P4[0] - np.cos(Theta)*d, P4[1] - np.sin(Theta)*d)
    subpath1 = straight_path(P3, P4)

    path2 = concatenate_paths(subpath1, subpath2, subpath3)

    return path1, path2

p1, p2 = build_reference_path(30)
assert p1.length == p2.length
np.testing.assert_array_almost_equal(p1(p1.length), p2(p2.length))
np.testing.assert_array_almost_equal(p1(0.5*p1.length), p2(0.5*p2.length))

def collision_set(path1, path2, threshold=0.99, ds=0.1):
    def extend_collision_enveloppe(s1, s2):
        collision_point = []
        if np.linalg.norm(path1(s1) - path2(s2)) < threshold:
            if np.linalg.norm(path1(s1 + ds) - path2(s2)) >= threshold:
                collision_point.append((s1 + ds, s2))
            if np.linalg.norm(path1(s1 - ds) - path2(s2)) >= threshold:
                collision_point.append((s1 - ds, s2))
            if np.linalg.norm(path1(s1) - path2(s2 + ds)) >= threshold:
                collision_point.append((s1, s2 + ds))
            if np.linalg.norm(path1(s1) - path2(s2 + ds)) >= threshold:
                collision_point.append((s1, s2 - ds))
            if collision_point:
                collision_point.append((s1, s2))
        return collision_point
    result = [point for point in [extend_collision_enveloppe(s1, s2) 
                                  for s1 in np.arange(0., path1.length + ds, ds)
                                  for s2 in np.arange(0., path2.length + ds, ds)] if point]
    return None if not result else np.array(reduce(lambda x,y: x+y, result))

def plot(path, fig=None, ax=None, resolution=100, decorate=False):
    if not ax: ax = plt.gca()
    if not fig: fig = plt.gcf()
    s = np.linspace(0, path.length, resolution)
    if not decorate:
        line = ax.plot(path(s)[:, 0], path(s)[:, 1])
    else:
        line_width = 20
        line1, = ax.plot(path(s)[:, 0], path(s)[:, 1], 'k-', lw=line_width)
        line2, = ax.plot(path(s)[:, 0], path(s)[:, 1], 'w-', lw=line_width-2)
        line3, = ax.plot(path(s)[:, 0], path(s)[:, 1], 'k-', lw=line_width-4)
        line4, = ax.plot(path(s)[:, 0], path(s)[:, 1], 'w--', lw=1)
        line = (line1, line2, line3, line4)
    ax.set_aspect('equal')
    return line

#p1 = straight_path(np.r_[-5, 2], np.r_[5, 2])
#p2 = concatenate_paths(straight_path(np.r_[-5, np.tan(np.pi/6)*(-5+1) + np.sin(np.pi/3)],
#                                     np.r_[-1, 2*np.sin(np.pi/3)]),
#                       circular_path(0, 2, 2*np.pi/3, np.pi/2),
#                       straight_path(np.r_[0, 2], np.r_[5, 2]))
#plot(p1)
#plot(p2)
#
#collision = collision_set(p1, p2)
#from scipy.spatial import ConvexHull
#convex = ConvexHull(collision)
#plt.figure()
#plt.plot(collision[:, 0], collision[:, 1], 'b+')
#for simplex in convex.simplices:
#    plt.plot(collision[simplex, 0], collision[simplex, 1], 'k-')
#plt.axis('equal')
#plt.show()
