import matplotlib.pyplot as plt
import numpy as np

import data as cppdata

import pandas as pd
hmap = pd.read_csv('../data/highway_map.csv', names=['x', 'y', 's', 'dx', 'dy'], sep=' ')

def show(x, y, t, s, d):
    fig, (ax, bx) = plt.subplots(figsize=(16,9), ncols=2)
    dx = x[-1] - x[0]
    dy = y[-1] - y[0]
    dist = np.sqrt(dx**2 + dy**2)
    ax.set_title('span is %.2f meters' % dist)
    ax.plot(x, y);
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    bx.set_xlabel('t')
    bx.plot(t, x, color='red')
    bx.set_ylabel('x', color='red')
    cx = bx.twinx()
    cx.plot(t, y, color='blue')
    cx.set_ylabel('y', color='blue')

    for a in bx, cx:
        a.grid(False)
    fig.tight_layout();

    '========'

    fig, (ax, bx) = plt.subplots(figsize=(16,9), ncols=2)
    dx = np.diff(x)
    dy = np.diff(y)
    spd = np.sqrt(dx**2 + dy**2) / .02
    ax.plot(t[:-1], spd)
    spd[0], spd[-1]

    bx.plot(np.array(s) - min(s), d)
    bx.set_xlabel('s')
    bx.set_ylabel('d')

    fig.tight_layout()

keys = list(sorted(cppdata.data.keys()))
k = keys[5]
d = cppdata.data[k]
show(d['x'], d['y'], d['t'], d['s'], d['d'])

SHOW_PLANS = True
SHOW_ACTUAL = True
MIN_K = 0
MAX_K = 2#-20

fig, ax = plt.subplots(figsize=(16,9))
for k in keys[MIN_K:MAX_K]:
    d = cppdata.data[k]
    if SHOW_PLANS:
        ax.plot(
            d['x'], d['y'],
            color='black', alpha=.5, linestyle='-',
        )
    if SHOW_ACTUAL and len(d['x_prev']) > 0:
        for which in 0, -1:
            ax.scatter(
                d['x_prev'][which], d['y_prev'][which],
                color='red' if which else 'green',
            )
        ax.plot(
            d['x_prev'], d['y_prev'],
            color='red', alpha=.5, linestyle='-',
        )
xl = ax.get_xlim()
yl = ax.get_ylim()
for k in keys[MIN_K:MAX_K:]:
    d = cppdata.data[k]
    n = np.array(d['neighbors']).T # (n, 4)
    ax.quiver(n[0], n[1], n[2], n[3], scale=500, width=.001, alpha=.1, color='cyan')
ax.set_xlim(xl)
ax.set_ylim(yl)

ax.plot(
    hmap['x'], hmap['y'], 
    color='yellow', marker='o',
)
ax.quiver(
    hmap['x'], hmap['y'], hmap['dx'], hmap['dy'], 
    scale=400, width=.002, alpha=.5, color='yellow'
)

# ax.set_xlim(1100, 1350)
# ax.set_ylim(1170, 1205)
ax.set_xlim(900, 945)
ax.set_ylim(1122, 1136)

ax.set_xlabel('$x$ [m] (world coordinates)')
ax.set_ylabel('$y$ [m] (world coordinates)')

ax.grid(False)


plt.show()

