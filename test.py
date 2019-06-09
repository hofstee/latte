import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import itertools

np.set_printoptions(suppress=True, precision=2)

len0 = 128.98
len1 = 83.83
len2 = 69.97
len3 = 140.0

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)

def lines(deg0, deg1, deg2):
    print(f"lines {deg0, deg1, deg2}")
    p0 = np.array((0, 0))
    p1 = np.around(p0 + len0 * np.array((np.cos(np.radians(deg0)), np.sin(np.radians(deg0)))), decimals=2)
    p2 = np.around(p1 + len1 * np.array((np.cos(np.radians(deg1)), np.sin(np.radians(deg1)))), decimals=2)
    p3 = np.around(p2 + len2 * np.array((np.cos(np.radians(deg2 + 90)), np.sin(np.radians(deg2 + 90)))), decimals=2)
    p4 = np.around(p3 + len3 * np.array((np.cos(np.radians(deg2)), np.sin(np.radians(deg2)))), decimals=2)

    return pairwise([p0, p1, p2, p3, p4])

c = np.array([
    (1, 0, 0, 1),
    (0, 1, 0, 1),
    (0, 0, 1, 1),
    (0, 0, 1, 1),
])

def move_to(x, y, pan, tilt, l=None):
    fig, ax = pl.subplots()
    if l is None:
        l = pairwise([(x,y),
                      (x - len3 * np.cos(np.radians(pan)) * np.sin(np.radians(tilt)), y - len3 * np.sin(np.radians(pan)) * np.sin(np.radians(tilt))),
                      (x + len2 * np.sin(np.radians(pan)) - len3 * np.cos(np.radians(pan)) * np.sin(np.radians(tilt)), y - len2 * np.cos(np.radians(pan)) - len3 * np.sin(np.radians(pan)) * np.sin(np.radians(tilt))),
                      (0,0),
        ])
    lc = mc.LineCollection(l, colors='b', linewidths=4)
    ax.add_collection(lc)

    sin_pan = np.sin(np.radians(pan))
    cos_pan = np.cos(np.radians(pan))

    sin_tilt = np.sin(np.radians(tilt))

    offset_x = -len2 * sin_pan + len3 * cos_pan * sin_tilt
    offset_y = len2 * cos_pan + len3 * sin_pan * sin_tilt

    target_x = x - offset_x
    target_y = y - offset_y
    print(offset_x, offset_y)
    print(target_x, target_y)

    ax.scatter(target_x, target_y, s=80, facecolors='none', edgecolors='r')

    R = np.sqrt(target_x**2 + target_y**2)
    gamma = np.arctan2(target_y, target_x)
    beta = np.arccos(((R**2 - len0**2 - len1**2) / (-2 * len0 * len1) - 1.0) % 2.0 - 1.0)
    psi = np.pi - beta
    alpha = np.arcsin((len1 * np.sin(psi)) / np.sqrt(R**2))

    print("rhs", gamma - alpha, np.pi - beta, gamma - alpha + psi + pan)
    l2 = lines(np.degrees(gamma - alpha), np.degrees(gamma - alpha + psi), pan)
    lc = mc.LineCollection(l2, colors='r', linewidths=2)
    ax.add_collection(lc)

    print("lhs", gamma + alpha, -np.pi + beta, gamma + alpha - psi + pan)
    l3 = lines(np.degrees(gamma + alpha), np.degrees(gamma + alpha - psi), pan)
    lc = mc.LineCollection(l3, colors='g', linewidths=2)
    ax.add_collection(lc)

    ax.set_xlim((-(len0+len1+len3+10), len0+len1+len3+10))
    ax.set_ylim((0,len0+len1+len3+10))
    ax.margins(0.1)

    plt.show(ax)

l = lines(57.15, 90, 90)
move_to(0, 332.19, 90, 90, l=l)

l = lines(115.82, 180, 0)
move_to(0, 186.08, 0, 90, l=l)

l = lines(164.84, 164.84-(180-115.82), 0)
move_to(0, 186.08, 0, 90, l=l)

l = list(lines(15, 105, 180))
x,y = tuple(l[-1][1])
print(x, y)
move_to(x, y, 180, 90, l=l)

# lc = mc.LineCollection(l, colors=c, linewidths=2)
# fig, ax = pl.subplots()
# ax.add_collection(lc)

# ax.set_xlim((-(len0+len1+len3+10), len0+len1+len3+10))
# ax.set_ylim((0,len0+len1+len3+10))
# ax.margins(0.1)

# plt.show(ax)
