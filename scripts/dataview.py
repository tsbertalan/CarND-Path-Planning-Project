from __future__ import print_function

import matplotlib.pyplot as plt
import numpy as np
import pandas as pd

import data as cppdata

hmap = pd.read_csv('../data/highway_map.csv', names=['x', 'y', 's', 'dx', 'dy'], sep=' ')

keys = cppdata.data.keys()
print(len(keys) - 1, 'plans saved.')

CAR_WIDTH = 3.
CAR_LENGTH = 6.
WWARN_RADIUS = CAR_WIDTH - .5
LWARN_RADIUS = CAR_LENGTH + 4

class Segment(object):

    def __init__(self, datadict):
        # ['t_pin_0', 't_pin_1', 'sdy_xyy_t', 't_responsible_1', 't_responsible_2']
        self.__dict__.update(datadict)

    def show(self, ax, **kw):
        kw.setdefault('color', 'black')
        kw.setdefault('linestyle', '-')
        kw.setdefault('marker', 'o')
        kw.setdefault('markersize', 2)
        s, d, yawf, x, y, yaww, t = self.sdy_xyy_t.T
        ax.plot(x, y, **kw)
        ax.scatter(x[0], y[0], edgecolor=kw['color'], facecolor=kw['color'], s=42)
        ax.scatter(x[-1], y[-1], edgecolor=kw['color'], facecolor='none', s=42)

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


for k in sorted(keys)[::-1]:
    if k == 'map':
        continue

    fig, ax = plt.subplots(figsize=(32,18))

    d = cppdata.data[k]

    num_unused = d['num_unused']
    last_plan_length = d['last_plan_length']
    num_used = last_plan_length - num_unused

    xyyt = d['plan_xyyt']
    x, y, yaw, t = xyyt.T
    
    # x_plan = x[num_used:]
    # y_plan = y[num_used:]
    # ax.plot(
    #     x_plan, y_plan,
    #     color='black', alpha=.5, linestyle='-', marker='o', markersize=1,
    #     label='new plan'
    # )

    segments = [Segment(segment_data) for segment_data in d['plan']['dumps']['segments']]
    for segment in segments:
        segment.show(ax)

    def reduce_lims(ax, which='y', fractions=[.1, .3, .6, .9]):
        lo, hi = getattr(ax, 'get_%slim' % which)()
        d = hi - lo
        locs = [f * d + lo for f in fractions]
        getattr(ax, 'set_%sticks' % which)(locs)

    # Inset plot for original Frenet coordinates
    #                 l   b   w   h
    inset = plt.axes([.2, .5, .2, .2])
    sdyt = d['plan_sdyt'].T
    sdyt[0] -= sdyt[0].min()
    inset.plot(sdyt[0], sdyt[1], color='green')
    inset.set_xlabel('$s$'); inset.set_ylabel('$d$')
    for i in 0, 4, 8, 12:
        edge = i not in (0, 12)
        inset.axhline(i, linestyle='--' if edge else '-', color='gold' if not i else 'black')
    inset.set_yticks([0, 4, 8, 12])
    inset.set_xticks(np.linspace(sdyt[0].min(), sdyt[0].max(), 4))
    # inset.set_xticklabels(inset.get_xticklabels())#, fontsize=12)
    # inset.set_yticklabels(inset.get_yticklabels(), fontsize=12)
    inset.tick_params(axis='both', which='major', labelsize=10)
    inset.grid(False)
    yl = inset.get_ylim()
    inset.set_ylim(max(14, yl[1]), min(-2, yl[0]))
    inset.set_xlim(sdyt[0,0], sdyt[0,-1])
    inset.set_title('as planned\n(before x,y transform)', fontsize=16, color='green')
    
    # Calculate velocity, accel, and jerk.
    D = np.diff

    i1 = 28
    i2 = 40

    i1 = 7
    i2 = 15
    
    i1 = 0
    i2 = -1

    # i1 = 38

    xyyt = d['plan_xyyt']
    x, y, yaw, t = xyyt[i1:i2].T
    xname, yname = 'x', 'y'


    sdyt = np.array(d['plan_sdyt']).T
    x = sdyt[0]
    # x -= x.min()
    y = sdyt[1]
    # yaw=sdyt[2]
    t = sdyt[3]
    xname, yname = 's', 'd'

    # tplot = np.arange(len(t))
    # tname = '$i_t$'
    tplot = t
    tname = '$t$ [s]'

    # Inset plot for raw x and y values
    inset = plt.axes([.7, .6, .2, .2])
    inset.set_xlabel('$i_t$')
    inset.plot(tplot, x, color='black')#, marker='o')
    inset.set_ylabel('$%s$' % xname)
    inset2 = inset.twinx()
    inset2.plot(tplot, y, color='red')#, marker='o')
    inset2.tick_params(axis='y', colors='red')
    inset2.set_ylabel('$%s$' % yname, color='red')
    inset.grid(False)
    inset2.grid(False)
    inset.set_xlim(min(tplot), max(tplot))
    inset.set_xticks([])
    reduce_lims(inset); reduce_lims(inset2); 

    # if len(d['prev']['t']) > 0: inset.axvline(len(d['prev']['t'])-1, color='purple', label='last previous point')
    inset.legend(loc='best')

    # Inset plot for split x and y velocities
    inset = plt.axes([.7, .4, .2, .2])
    dt = D(t)
    dx = D(x)
    dy = D(y)
    print(dx/dt)
    inset.set_xlabel('$i_t$')
    inset.plot(tplot[1:], dx/dt, color='black')
    inset.set_ylabel(r'$\dot %s$ [$m/s$]' % xname)
    inset2 = inset.twinx()
    inset2.plot(tplot[1:], dy/dt, color='red')
    inset2.tick_params(axis='y', colors='red')
    inset2.set_ylabel(r'$\dot %s$ [$m/s$]' % yname, color='red')
    inset.grid(False)
    inset2.grid(False)
    inset.set_xlim(min(tplot), max(tplot))
    inset.set_xticks([])
    reduce_lims(inset); reduce_lims(inset2); 
    # if len(d['prev']['t']) > 0: inset.axvline(len(d['prev']['t'])-1, color='purple', label='last previous point')

    # Inset plot for acceleration and jerk profiles
    inset = plt.axes([.7, .2, .2, .2])

    a_x = D(dx/dt) / dt[:-1]
    a_y = D(dy/dt) / dt[:-1]

    a = np.sqrt(a_x**2 + a_y**2)
    jerk = D(a) / dt[:-2]

    inset.set_xlabel(tname)

    inset.plot(tplot[2:], a, color='black')
    inset.set_ylabel('$||$acceleration$||$\n[$m/s^2$]')

    inset2 = inset.twinx()
    inset2.plot(tplot[3:], jerk, color='red')
    inset2.tick_params(axis='y', colors='red')
    inset2.set_ylabel('jerk [$m/s^3$]', color='red')

    inset.grid(False)
    inset2.grid(False)
    inset.set_xlim(min(tplot), max(tplot))
    reduce_lims(inset); reduce_lims(inset2); 
    # if len(d['prev']['t']) > 0: inset.axvline(len(d['prev']['t'])-1, color='purple', label='last previous point')


    # Bounding boxes
    xyyt = d['plan_xyyt']
    x, y, yaw, t = xyyt.T
    t_reuse, t_replan = d['t_reuse'], d['t_replan']
    i_reuse = np.argmin(np.abs(t - t_reuse))
    i_replan = np.argmin(np.abs(t - t_replan))
    xcar = x[i_reuse]
    ycar = y[i_reuse]
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
        # if len(neighbor) > 5:
        #     nxend, nyend = neighbor[5], neighbor[6]
        #     ax.plot([nx, nxend], [ny, nyend], color='navy', linestyle='-', alpha=.75)

    # Global map things.
    xl = ax.get_xlim()
    yl = ax.get_ylim()
    ax.plot(
        cppdata.data['map']['map_x'], cppdata.data['map']['map_y'],
        color='gold',
        label='map centerline',
        marker='o', markersize=2,
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
    ax.legend(loc='lower left')
    fig.suptitle(
        '"%s"' % d['plan']['desc'],
        fontsize=32,
    )
    ax.set_title(
        '%s segment(s). %s\nPlanned in %d [ms] %s.' % (
            len(d['plan']['dumps']['segments']),
            '$t=%.3f$ [s]' % (k/1000.), 
            d['planner_time'], 
            d['reasons']
            ),
        fontsize=18,
    )


    plt.show()
    # inp = raw_input('Q to quit or blank to continue:')
    # if inp.lower() == 'q':
    #     break
