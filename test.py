import numpy as np
import pylab as pl
import matplotlib.pyplot as plt
from matplotlib import collections as mc
import itertools


np.set_printoptions(suppress=True, precision=2)


def config1():
    return 128.98, 83.83, 69.97, 140.0


def config2():
    return 128.98, 83.83, 69.97, 100.0


def config3():
    return 128.98, 83.83, 101.97, 45.0, 15.0


# len0, len1, len2, len3 = config2()
len0, len1, len2, len3, len4 = config3()


def y_extent(tilt):
    if tilt == 90:
        y_min = 135.4 if len3 == 45 else 206.7
    elif tilt == 0:
        y_min = None
    y_max = len0 * np.sin(np.arccos(len2 / len0)) + len1 + len3 * np.sin(np.radians(tilt))
    return y_min, y_max


print(y_extent(90))
print(y_extent(0))

def pairwise(iterable):
    "s -> (s0,s1), (s1,s2), (s2, s3), ..."
    a, b = itertools.tee(iterable)
    next(b, None)
    return zip(a, b)


def lines(deg0, deg1, deg2, deg3=90):
    print(f"lines {deg0, deg1, deg2}")
    p0 = np.array((0, 0))
    p1 = np.around(p0 + len0 * np.array((np.cos(np.radians(deg0)), np.sin(np.radians(deg0)))), decimals=2)
    p2 = np.around(p1 + len1 * np.array((np.cos(np.radians(deg1)), np.sin(np.radians(deg1)))), decimals=2)
    p3 = np.around(p2 + len2 * np.array((np.cos(np.radians(deg2 + 90)), np.sin(np.radians(deg2 + 90)))), decimals=2)
    p4 = np.around(p3 + len3 * np.sin(np.radians(deg3)) * np.array((np.cos(np.radians(deg2)), np.sin(np.radians(deg2)))), decimals=2)

    return pairwise([p0, p1, p2, p3, p4])


def move_to(x, y, pan, tilt, l=None):
    # print("===")
    # fig, ax = pl.subplots()
    # if l is None:
    #     l = pairwise([(x,y),
    #                   (x - len3 * np.cos(np.radians(pan)) * np.sin(np.radians(tilt)), y - len3 * np.sin(np.radians(pan)) * np.sin(np.radians(tilt))),
    #                   (x + len2 * np.sin(np.radians(pan)) - len3 * np.cos(np.radians(pan)) * np.sin(np.radians(tilt)), y - len2 * np.cos(np.radians(pan)) - len3 * np.sin(np.radians(pan)) * np.sin(np.radians(tilt))),
    #                   (0,0),
    #     ])
    # lc = mc.LineCollection(l, colors='b', linewidths=4)
    # ax.add_collection(lc)

    sin_pan = np.sin(np.radians(pan))
    cos_pan = np.cos(np.radians(pan))

    sin_tilt = np.sin(np.radians(tilt))
    cos_tilt = np.cos(np.radians(tilt))

    offset_x = -len2 * sin_pan + len3 * cos_pan * sin_tilt + len4 * cos_pan * cos_tilt
    offset_y = len2 * cos_pan + len3 * sin_pan * sin_tilt + len4 * sin_pan * cos_tilt

    target_x = x - offset_x
    target_y = y - offset_y
    # print(offset_x, offset_y)
    # print(target_x, target_y)

    # ax.scatter(target_x, target_y, s=80, facecolors='none', edgecolors='r')

    R = np.sqrt(target_x**2 + target_y**2)
    gamma = np.arctan2(target_y, target_x)
    beta = np.arccos(((R**2 - len0**2 - len1**2) / (-2 * len0 * len1) - 1.0) % 2.0 - 1.0)
    psi = np.pi - beta
    alpha = np.arcsin((len1 * np.sin(psi)) / np.sqrt(R**2))
    # print(R, gamma, beta, psi, alpha)
    # print((R**2 - len0**2 - len1**2) / (-2 * len0 * len1))

    # print("rhs", np.degrees(gamma - alpha), np.degrees(psi), np.degrees(gamma - alpha + psi) + pan)
    # l2 = lines(np.degrees(gamma - alpha), np.degrees(gamma - alpha + psi), pan, tilt)
    # lc = mc.LineCollection(l2, colors='r', linewidths=2)
    # ax.add_collection(lc)

    # print("lhs", np.degrees(gamma + alpha), np.degrees(-psi), np.degrees(gamma + alpha - psi) + pan)
    # l3 = lines(np.degrees(gamma + alpha), np.degrees(gamma + alpha - psi), pan, tilt)
    # lc = mc.LineCollection(l3, colors='g', linewidths=2)
    # ax.add_collection(lc)

    # ax.set_xlim((-(len0+len1+len3+10), len0+len1+len3+10))
    # ax.set_ylim((0,len0+len1+len3+10))
    # ax.margins(0.1)

    # plt.show(ax)

    return np.degrees(gamma - alpha), 90 + psi, 270 - (gamma - alpha + 90 + psi) + pan


def heart(num_rotations=7, fill_dur=5000, heart_dur=5000, pull_dur=1000):
    x = []
    y = []
    pan = []
    tilt = []

    start_tilt = 90
    end_tilt = 30
    it = range(0, fill_dur)
    x += [15 * np.cos(np.interp(ms, [0, fill_dur], [0, num_rotations*2*np.pi])) for ms in it]
    y += [150 + 10 * np.sin(np.interp(ms, [0, fill_dur], [0, num_rotations*2*np.pi])) for ms in it]
    pan += [90 for _ in it]
    tilt += [np.interp(ms, [0, fill_dur], [start_tilt, end_tilt]) for ms in it]

    # start_tilt = end_tilt
    # end_tilt = start_tilt + 15
    # it = range(0, 500)
    # x += [0 for _ in it ]
    # y += [130 for _ in it ]
    # pan += [90 for _ in it ]
    # tilt += [np.interp(ms, [0, 500], [start_tilt, end_tilt]) for ms in it ]

    # start_tilt = start_tilt
    # end_tilt = start_tilt - 20
    # it = range(0, heart_dur)
    # x += [0 for _ in it ]
    # y += [130 for _ in it ]
    # pan += [90 for _ in it ]
    # tilt += [np.interp(ms, [0, heart_dur], [start_tilt, end_tilt]) for ms in it ]

    # start_tilt = end_tilt
    # end_tilt = start_tilt - 10
    # it = range(0, pull_dur)
    # x += [0 for _ in it]
    # y += [130 + np.interp(ms, [0, pull_dur], [0, 40]) for ms in it]
    # pan += [90 for _ in it]
    # tilt += [np.interp(ms, [0, pull_dur], [start_tilt, end_tilt]) for ms in it]

    # start_tilt = end_tilt
    # end_tilt = 90
    # it = range(0, 2000)
    # x += [0 for _ in it]
    # y += [130 + np.interp(ms, [0, 2000], [40, 0]) for ms in it]
    # pan += [90 for _ in it]
    # tilt += [np.interp(ms, [0, 2000], [start_tilt, end_tilt]) for ms in it]

    return x, y, pan, tilt


x, y, pan, tilt = heart()
d0s = []
d1s = []
for k in range(len(x)):
    d0, d1, d2 = move_to(x[k], y[k], pan[k], tilt[k])
    d0s.append(d0)
    d1s.append(d1)

print(d0s[0:715])
print(d1s[0:715])
# print(pan[::100])
# print(tilt[::100])

# l = lines(57.15, 90, 90)
# move_to(0, 332.19, 90, 90, l=l)

# l = lines(115.82, 180, 0)
# move_to(0, 186.08, 0, 90, l=l)

# l = lines(164.84, 164.84-(180-115.82), 0)
# move_to(0, 186.08, 0, 90, l=l)

# pan = 90
# l = list(lines(0, 90, pan))
# x,y = tuple(l[-1][1])
# print(x, y)
# move_to(x, y, pan, 90, l=l)

# move_to(0, 200, 90, 0)

# l = list(lines(15, 105, 180))
# move_to(-37.11, 44.38, 180, 90, l=l)

# lc = mc.LineCollection(l, colors=c, linewidths=2)
# fig, ax = pl.subplots()
# ax.add_collection(lc)

# ax.set_xlim((-(len0+len1+len3+10), len0+len1+len3+10))
# ax.set_ylim((0,len0+len1+len3+10))
# ax.margins(0.1)

# plt.show(ax)
