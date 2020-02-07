#!/usr/bin/env python3

"""
Analyze and plot 2d motion with drag

Try to run a straight line from (x0, y0) to (x1, y1) with velocity target_v
How do I control thrust to achieve this?

TODO implement a velocity ramp down in last segment
TODO give drag coef appropriate body frame names, e.g., FORWARD, LEFT
TODO model assumes sub faces towards +y, change this so sub spins while moving in a straight line
"""

from math import cos, sin, atan2

import matplotlib.pyplot as plt


class Plotter(object):
    """Plot a bunch of stuff"""

    def __init__(self):
        # Drag accelerations
        self.daxs = []
        self.days = []

        # Thrust efforts
        self.exs = []
        self.eys = []

        # Total accelerations
        self.axs = []
        self.ays = []

        # Velocities
        self.vxs = []
        self.vys = []

        # Positions
        self.xs = []
        self.ys = []

        # Times
        self.ts = []

    def plot(self):

        fig, (pe, pa, pv, pp) = plt.subplots(4, sharex='all')

        pe.set_title('thrust effort vs time')
        pe.plot(self.ts, self.exs, label='effort x')
        pe.plot(self.ts, self.eys, label='effort y')
        pe.legend()

        pa.set_title('acceleration vs time')
        pa.plot(self.ts, self.axs, label='ax')
        pa.plot(self.ts, self.ays, label='ay')
        pa.plot(self.ts, self.daxs, label='drag accel x')
        pa.plot(self.ts, self.days, label='drag accel y')
        pa.legend()

        pv.set_title('velocity vs time')
        pv.plot(self.ts, self.vxs, label='vx')
        pv.plot(self.ts, self.vys, label='vy')
        pv.legend()

        pp.set_title('position vs time')
        pp.plot(self.ts, self.xs, label='x')
        pp.plot(self.ts, self.ys, label='y')
        pp.legend()

        plt.show()


MASS = 1.
BOLLARD_FORCE = 1.

DRAG_COEF_X = 0.1       # If body is facing +y in the world frame then this is DRAG_COEF_LEFT
DRAG_COEF_Y = 0.2       # If body is facing +y in the world frame then this is DRAG_COEF_FORWARD

TARGET_V = 1.           # Target velocity
DT = 0.1


def accel_to_force(a):
    return a * MASS


def force_to_effort(f):
    return f / BOLLARD_FORCE


def accel_to_effort(a):
    return force_to_effort(accel_to_force(a))


def force_to_accel(f):
    return f / MASS


def drag_force(v, coef):
    return coef * v * v


def drag_accel(v, coef):
    return -force_to_accel(drag_force(v, coef))


def constant_thrust(plotter: Plotter, x0, y0, x1, y1):

    angle_to_goal = atan2(y1 - y0, x1 - x0)
    print('angle to goal', angle_to_goal)

    target_vx = cos(angle_to_goal) * TARGET_V
    target_vy = sin(angle_to_goal) * TARGET_V
    print('target_vx', target_vx)
    print('target_vy', target_vy)

    # Feedforward == acceleration-due-to-thrust = drag at target velocity
    ffx = -drag_accel(target_vx, DRAG_COEF_X)
    ffy = -drag_accel(target_vy, DRAG_COEF_Y)
    print('ffx', ffx)
    print('ffy', ffy)

    # Start velocity
    vx = 0.
    vy = 0.

    # Start position
    x = x0
    y = y0

    # Start time
    t = 0

    for i in range(200):
        t += DT

        plotter.ts.append(t)

        # Acceleration due to drag
        dax = drag_accel(vx, DRAG_COEF_X)
        day = drag_accel(vy, DRAG_COEF_Y)

        plotter.daxs.append(dax)
        plotter.days.append(day)

        # Thrust effort
        ex = accel_to_effort(ffx)
        ey = accel_to_effort(ffy)

        plotter.exs.append(ex)
        plotter.eys.append(ey)

        # Total acceleration
        ax = ffx + dax
        ay = ffy + day

        plotter.axs.append(ax)
        plotter.ays.append(ay)

        # Velocity
        vx += ax * DT
        vy += ay * DT

        plotter.vxs.append(vx)
        plotter.vys.append(vy)

        # Position
        x += vx * DT
        y += vy * DT

        plotter.xs.append(x)
        plotter.ys.append(y)


def velocity_ramp(plotter: Plotter, x0, y0, x1, y1):

    angle_to_goal = atan2(y1 - y0, x1 - x0)
    print('angle to goal', angle_to_goal)

    target_vx = cos(angle_to_goal) * TARGET_V
    target_vy = sin(angle_to_goal) * TARGET_V
    print('target_vx', target_vx)
    print('target_vy', target_vy)

    # Velocity ramp length, in seconds
    velocity_ramp_t = 5.

    # Total acceleration during the velocity ramp
    ax = target_vx / velocity_ramp_t
    ay = target_vy / velocity_ramp_t

    # Start velocity
    vx = 0.
    vy = 0.

    # Start position
    x = x0
    y = y0

    # Start time
    t = 0

    for i in range(200):
        t += DT

        plotter.ts.append(t)

        # Total acceleration is fixed
        plotter.axs.append(ax)
        plotter.ays.append(ay)

        # Acceleration due to drag
        dax = drag_accel(vx, DRAG_COEF_X)
        day = drag_accel(vy, DRAG_COEF_Y)

        plotter.daxs.append(dax)
        plotter.days.append(day)

        # Acceleration due to thrust
        tax = ax - dax
        tay = ay - day

        # Acceleration due to thrust must be > 0
        assert tax >= 0.
        assert tay >= 0.

        # Thrust effort
        ex = accel_to_effort(tax)
        ey = accel_to_effort(tay)

        plotter.exs.append(ex)
        plotter.eys.append(ey)

        # Velocity
        vx += ax * DT
        vy += ay * DT

        plotter.vxs.append(vx)
        plotter.vys.append(vy)

        # When we hit target velocity, set acceleration = 0
        # This happens at same time!
        if vx >= target_vx:
            ax = 0
        if vy >= target_vy:
            ay = 0

        # Position
        x += vx * DT
        y += vy * DT

        plotter.xs.append(x)
        plotter.ys.append(y)


def main():
    plotter = Plotter()

    constant_thrust(plotter, 0., 0., 10., 10.)
    # velocity_ramp(plotter, 0., 0., 10., 10.)

    plotter.plot()


if __name__ == '__main__':
    main()
