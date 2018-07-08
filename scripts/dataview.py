import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
from scipy import interpolate

import data as cppdata

hmap = pd.read_csv('../data/highway_map.csv', names=['x', 'y', 's', 'dx', 'dy'], sep=' ')


def show(x, y, t, s, d, x_prev, y_prev):
    fig, (ax, bx) = plt.subplots(figsize=(16,9), ncols=2)
    dx = x[-1] - x[0]
    dy = y[-1] - y[0]
    dist = np.sqrt(dx**2 + dy**2)
    ax.set_title('span is %.2f meters' % dist)
    ax.plot(x, y, label='plan');
    ax.scatter(x_prev, y_prev, label='previous')
    ax.set_xlabel('x')
    ax.set_ylabel('y')

    bx.set_xlabel('t')
    bx.plot(t, x, color='red')
    bx.set_ylabel('x', color='red')
    cx = bx.twinx()
    cx.plot(t, y, color='black')
    cx.set_ylabel('y', color='black')

    for a in bx, cx:
        a.grid(False)
    fig.tight_layout();

    ax.legend(loc='best')

    '========'

    fig, (ax, bx, cx) = plt.subplots(figsize=(16, 9), ncols=3)
    dx = np.diff(x)
    dy = np.diff(y)
    spd = np.sqrt(dx**2 + dy**2) / .02
    ax.plot(t[:-1], spd)
    ax.set_xlabel('$t$')
    ax.set_ylabel('speed')
    spd[0], spd[-1]

    bx.plot(np.array(s) - min(s), d, marker='o')
    bx.set_xlabel('s')
    bx.set_ylabel('d')
    nprev = len(x_prev)
    print((np.array(s) - min(s))[:nprev], d[:nprev])
    bx.scatter(
        (np.array(s) - min(s))[:nprev], d[:nprev],
        marker='o', facecolors='none', edgecolors='red', s=142,
        label='leftover',
        zorder=99)

    cx.plot(t, np.array(s) - min(s), color='black', marker='o')
    cx.set_xlabel('t', color='black')
    cx.set_ylabel('s')

    dx = cx.twinx()
    dx.plot(t, d, color='red', marker='o')
    dx.set_ylabel('d', color='red')
    spline_d = interpolate.UnivariateSpline(t[:nprev], d[:nprev], k=3, s=0)
    tfine = np.linspace(t[0], t[nprev - 1], 200)
    dx.plot(tfine, spline_d(tfine), color='magenta')
    dx.set_title("$d'=%f$,\n$d''=%f$" % (
        spline_d.derivative(1)(t[nprev - 1]),
        spline_d.derivative(2)(t[nprev - 1]),
    ), color='magenta')

    cx.grid(False)

    fig.tight_layout()

keys = list(sorted(cppdata.data.keys()))
for which_k in range(len(keys) - 1, -1, -1):
    k = keys[which_k]
    d = cppdata.data[k]

    show(d['x'], d['y'], d['t'], d['s'], d['d'], d['x_prev'], d['y_prev'])
    plt.show()
    inp = raw_input('Q to quit or blank to continue:')
    if inp.lower() == 'q':
        break

SHOW_PLANS = True
SHOW_ACTUAL = True
MIN_K = 0
MAX_K = -1

print(len(keys), 'plans saved.')

fig, ax = plt.subplots(figsize=(16,9))
dolabel1 = True
dolabel2 = True
for k in keys[MIN_K:MAX_K]:
    d = cppdata.data[k]

    xleftover = d['x_prev']
    yleftover = d['y_prev']
    x = d['x']
    y = d['y']

    assert (np.array(x[:len(xleftover)]) - xleftover == 0).all()
    assert (np.array(y[:len(yleftover)]) - yleftover == 0).all()
    
    if SHOW_PLANS:
        x = x[len(xleftover):]
        y = y[len(yleftover):]

        ax.plot(
            x, y,
            color='black', alpha=.5, linestyle='-',
            label='new plan' if dolabel1 else None,
        )
        dolabel1 = False

    if SHOW_ACTUAL and len(d['x_prev']) > 0:
        for which in 0, -1:
            ax.scatter(
                d['x_prev'][which], d['y_prev'][which],
                color='red' if which else 'green',
            )
        ax.plot(
            d['x_prev'], d['y_prev'],
            color='red', alpha=.5, linestyle='-',
            label='leftover' if dolabel2 else None,
        )
        dolabel2 = False
xl = ax.get_xlim()
yl = ax.get_ylim()
for k in keys[MIN_K:MAX_K:]:
    d = cppdata.data[k]
    n = np.array(d['neighbors']).T # (n, 4)
    ax.quiver(n[0], n[1], n[2], n[3], scale=500, width=.001, alpha=.1, color='cyan', angles='xy')
ax.set_xlim(xl)
ax.set_ylim(yl)

ax.plot(
    hmap['x'], hmap['y'], 
    color='yellow', marker='o',
    label='map centerline'
)
ax.quiver(
    hmap['x'], hmap['y'], hmap['dx'], hmap['dy'],
    scale=400, width=.002, alpha=.5, color='yellow', angles='xy',
)

# ax.set_xlim(1100, 1350)
# ax.set_ylim(1170, 1205)
# ax.set_xlim(900, 945)
# ax.set_ylim(1122, 1136)

ax.set_xlabel('$x$ [m] (world coordinates)')
ax.set_ylabel('$y$ [m] (world coordinates)')

ax.grid(False)

ax.legend(loc='best')

plt.show()

