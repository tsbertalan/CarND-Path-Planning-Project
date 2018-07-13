from __future__ import print_function

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

if False:
    for which_k in range(len(keys) - 1, -1, -1):
        k = keys[which_k]
        if k == 'map':
            continue
        d = cppdata.data[k]

        show(d['plan']['x'], d['plan']['y'], d['plan']['t'], d['plan']['s'], d['plan']['d'], d['prev']['x'], d['prev']['y'])
        plt.show()
        inp = raw_input('Q to quit or blank to continue:')
        if inp.lower() == 'q':
            break

SHOW_PLANS = True
SHOW_ACTUAL = True
MIN_K = 0
MAX_K = -1

print(len(keys), 'plans saved.')

CAR_WIDTH = 3.
CAR_LENGTH = 6.
WWARN_RADIUS = CAR_WIDTH - .5
LWARN_RADIUS = CAR_LENGTH + 4

def draw_box(ax, x, y, theta=0, 
    width=CAR_WIDTH, length=CAR_LENGTH, 
    pad=(WWARN_RADIUS-CAR_WIDTH/2., LWARN_RADIUS-CAR_LENGTH/2.), **style):
    style.setdefault('linestyle', '-')

    wrad = width / 2.
    lrad = length / 2.

    X = x - lrad, x + lrad, x + lrad, x - lrad, x - lrad
    Y = y + wrad, y + wrad, y - wrad, y - wrad, y + wrad

    import matplotlib.transforms as transforms

    t = transforms.Affine2D().rotate_around(x, y, theta)
    U, V = t.get_matrix().dot(np.vstack([X, Y, np.ones(5)]))[:2]

    plots = [ax.plot(U, V, **style)]
    if pad is not None:
        padstyle = dict(style)
        padstyle['alpha'] = .25
        plots.extend(
            draw_box(ax, x, y, theta, width+pad[0]*2, length+pad[1]*2, pad=None, **padstyle)
        )
    return plots


for k in keys[::-1]:
    if k == 'map':
        continue

    fig, ax = plt.subplots(figsize=(32,18))
    dolabel1 = True
    dolabel2 = True

    d = cppdata.data[k]

    xleftover = d['prev']['x']
    yleftover = d['prev']['y']
    x = d['plan']['x']
    y = d['plan']['y']

    assert (np.array(x[:len(xleftover)]) - xleftover == 0).all()
    assert (np.array(y[:len(yleftover)]) - yleftover == 0).all()
    
    if SHOW_PLANS:
        x = x[len(xleftover):]
        y = y[len(yleftover):]

        ax.plot(
            x, y,
            color='black', alpha=.5, linestyle='-', marker='o', markersize=1,
            label='new plan' if dolabel1 else None,
        )
        dolabel1 = False

    if SHOW_ACTUAL and len(d['prev']['x']) > 0:
        for which in 0, -1:
            ax.scatter(
                d['prev']['x'][which], d['prev']['y'][which],
                color='red' if which else 'green',
            )
        ax.plot(
            d['prev']['x'], d['prev']['y'],
            color='red', alpha=.5, linestyle='-', marker='o', markersize=1,
            label='leftover' if dolabel2 else None,
        )
        dolabel2 = False

    # Inset plot for original Frenet coordinates
    #                 l   b   w   h
    inset = plt.axes([.2, .5, .2, .2])
    sdt = np.array(d['plan_sdt']).T
    sdt[0] -= sdt[0].min()
    inset.plot(sdt[0], sdt[1])
    inset.set_xlabel('$s$'); inset.set_ylabel('$d$')
    for i in 0, 4, 8, 12:
        inset.axhline(i, linestyle='--' if i not in (0, 12) else '-')
    inset.set_yticks([0, 4, 8, 12])
    inset.set_xticks(np.linspace(sdt[0].min(), sdt[0].max(), 4))
    # inset.set_xticklabels(inset.get_xticklabels())#, fontsize=12)
    # inset.set_yticklabels(inset.get_yticklabels(), fontsize=12)
    inset.tick_params(axis='both', which='major', labelsize=10)
    inset.grid(False)
    yl = inset.get_ylim()
    inset.set_ylim(max(14, yl[1]), min(-2, yl[0]))
    inset.set_xlim(sdt[0,0], sdt[0,-1])
    inset.set_title('as planned\n(before x,y transform)', fontsize=16)


    # Bounding boxes
    if len(xleftover) > 0:
        xcar = xleftover[0]
        ycar = yleftover[0]
    else:
        xcar = x[0]
        ycar = y[0]
    draw_box(ax, xcar, ycar, 
        theta=np.arctan2(y[1]-y[0], x[1]-x[0]),
        color='black'
    )

    # Show neighbor paths.
    n = np.array(d['neighbors']).T # (n, 4)
    ax.quiver(n[1], n[2], n[3], n[4], scale=500, width=.0025, alpha=.75, color='navy', angles='xy')

    for neighbor in n.T:
        nid, nx, ny, nvx, nvy = neighbor[:5]
        draw_box(ax, nx, ny, theta=np.arctan2(nvy, nvx), color='navy')
        ax.text(nx-1, ny-1, "#%d" % nid, fontsize=12)
        if len(neighbor) > 5:
            nxend, nyend = neighbor[5], neighbor[6]
            ax.plot([nx, nxend], [ny, nyend], color='navy', linestyle='-', alpha=.75)

    # Global map things.
    xl = ax.get_xlim()
    yl = ax.get_ylim()
    ax.plot(
        hmap['x'], hmap['y'], 
        color='yellow',
        label='map centerline'
    )

    lines = np.array(cppdata.data['map']['map_lines'])
    for i in range(6):
        l = np.array(lines[i]).T
        ax.plot(
            l[0], l[1], 
            color='black', alpha=.25,
            linestyle='--' if i not in (0, 5) else '-',
        )

    # ax.quiver(
    #     hmap['x'], hmap['y'], hmap['dx'], hmap['dy'],
    #     scale=50, width=.002, alpha=.95, color='yellow', angles='xy',
    # )


    # Final doctoring.
    # ax.set_xlim(xl)
    # ax.set_ylim(yl)
    ax.set_xlim(xcar-50, xcar+50)
    ax.set_ylim(ycar-50, ycar+50)
    ax.set_xlabel('$x$ [m] (world coordinates)')
    ax.set_ylabel('$y$ [m] (world coordinates)')
    ax.grid(False)
    ax.set_aspect('equal', 'datalim')
    ax.legend(loc='lower right')
    fig.suptitle(
        '"%s"' % d['plan']['desc'],
        fontsize=32,
    )
    ax.set_title(
        '%s\nPlanned in %d [ms] %s.' % ('$t=%.3f$ [s]' % (k/1000.), d['planner_time'], d['reasons']),
        fontsize=18,
    )


    plt.show()
    # inp = raw_input('Q to quit or blank to continue:')
    # if inp.lower() == 'q':
    #     break
