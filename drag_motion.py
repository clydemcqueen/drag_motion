#!/usr/bin/env python3

"""
Analyze and plot 2d motion with drag

Run in a straight line from (x0, y0) to (x1, y1) with velocity target_v
Yaw changes from yaw0 to yaw1

How do I control thrust effort (forward, strafe, yaw) to achieve this?
"""

from math import cos, sin, atan2, pi

import matplotlib.pyplot as plt


class Plotter(object):
    """Plot a bunch of stuff"""

    def __init__(self):
        # Yaw
        self.yaws = []

        # Thrust efforts in the body frame (forward, strafe)
        self.efs = []
        self.ess = []

        # Thrust accelerations (feedforwards)
        self.taxs = []
        self.tays = []

        # Drag accelerations
        self.daxs = []
        self.days = []

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
        fig, (py, pe, pa, pv, pp) = plt.subplots(5, sharex='all')

        py.set_title('yaw vs time')
        py.plot(self.ts, self.yaws, label='yaw')
        py.legend()

        pe.set_title('thrust effort vs time')
        pe.plot(self.ts, self.efs, label='effort forward')
        pe.plot(self.ts, self.ess, label='effort strafe')
        pe.legend()

        pa.set_title('acceleration vs time')
        pa.plot(self.ts, self.axs, label='ax')
        pa.plot(self.ts, self.ays, label='ay')
        pa.plot(self.ts, self.taxs, label='feedforward x')
        pa.plot(self.ts, self.tays, label='feedforward y')
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

DRAG_COEF_FORWARD = 0.1
DRAG_COEF_STRAFE = 0.2

TARGET_V = 1.  # Target velocity
DT = 0.1


def norm_angle(a: float) -> float:
    if a > pi:
        a -= 2 * pi
    elif a < -pi:
        a += 2 * pi
    return a


def rotate_frame(x, y, yaw):
    """Rotate world to body"""
    forward = x * cos(yaw) + y * sin(yaw)
    strafe = y * cos(yaw) - x * sin(yaw)
    return forward, strafe


def accel_to_force(a):
    return a * MASS


def force_to_effort(f):
    return f / BOLLARD_FORCE


def accel_to_effort(a):
    return force_to_effort(accel_to_force(a))


def force_to_accel(f):
    return f / MASS


def drag_force(v, coef):
    # Motion works in all 4 quadrants, note the use of abs()
    return v * abs(v) * -coef


def drag_accel(v, coef):
    return force_to_accel(drag_force(v, coef))


def drag_coef_world(yaw, motion_world):
    """
    Return drag coefficients in world frame: (drag_coef_x, drag_coef_y)

    yaw is the orientation of the body in the world frame
    motion_world is the direction of motion in the world frame
    If yaw == motion_world, the body is moving forward, so forward drag applies, etc.
    """

    # Direction of motion in the body frame
    motion_body = norm_angle(motion_world - yaw)

    # Fold quadrants II, II and IV into quadrant I
    if motion_body < 0:
        motion_body = -motion_body
    if motion_body > pi / 2:
        motion_body = pi - motion_body

    # How to combine forward and strafe drag coef? Will require some experimentation
    # For now, interpolate between _FORWARD and _STRAFE
    drag_coef_motion = (motion_body * DRAG_COEF_STRAFE + (pi / 2 - motion_body) * DRAG_COEF_FORWARD) / (pi / 2)

    # Break the drag down to x and y components
    drag_coef_x = cos(motion_world) * drag_coef_motion
    drag_coef_y = sin(motion_world) * drag_coef_motion

    # print('yaw', yaw / pi * 180., 'motion_world', motion_world / pi * 180., 'drag_coef_motion', drag_coef_motion,
    #       'drag_coef_x', drag_coef_x, 'drag_coef_y', drag_coef_y)

    return (drag_coef_x, drag_coef_y)


def constant_thrust(plotter: Plotter, x0, y0, x1, y1, yaw0):
    """
    Constant thrust (in the world frame), works well if forward drag coef == strafe drag coef
    Thrust in the body frame (forward, strafe) varies as yaw varies
    """

    angle_to_goal = atan2(y1 - y0, x1 - x0)
    print('angle to goal', angle_to_goal)

    target_vx = cos(angle_to_goal) * TARGET_V
    target_vy = sin(angle_to_goal) * TARGET_V
    print('target_vx', target_vx)
    print('target_vy', target_vy)

    # Feedforward == acceleration-due-to-thrust = drag at target velocity
    ffx = -drag_accel(target_vx, DRAG_COEF_FORWARD)
    ffy = -drag_accel(target_vy, DRAG_COEF_STRAFE)
    print('ffx', ffx)
    print('ffy', ffy)

    # Start velocity
    vx = 0.
    vy = 0.

    # Start position
    x = x0
    y = y0

    # Starting yaw
    yaw = yaw0

    # Start time
    t = 0

    for i in range(200):
        t += DT

        plotter.ts.append(t)

        # Spin around while moving
        yaw = norm_angle(yaw + 0.05)

        plotter.yaws.append(yaw / pi * 180.)

        # Acceleration due to drag
        drag_ax = drag_accel(vx, DRAG_COEF_FORWARD)
        drag_ay = drag_accel(vy, DRAG_COEF_STRAFE)

        plotter.taxs.append(ffx)
        plotter.tays.append(ffy)

        plotter.daxs.append(drag_ax)
        plotter.days.append(drag_ay)

        # Thrust effort
        effort_x = accel_to_effort(ffx)
        effort_y = accel_to_effort(ffy)

        # Rotate thrust from world frame to body frame
        effort_forward, effort_strafe = rotate_frame(effort_x, effort_y, yaw)

        plotter.efs.append(effort_forward)
        plotter.ess.append(effort_strafe)

        # Total acceleration
        ax = ffx + drag_ax
        ay = ffy + drag_ay

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


def constant_acceleration(plotter: Plotter, x0, y0, x1, y1, yaw0):
    """
    Constant acceleration, works well for any drag configuration

    TODO stop at x1, y1, yaw1 -- do this by time? Or distance?
    TODO vary yaw from yaw0 to yaw1
    TODO implement a velocity ramp down in last segment
    """

    # Direction of motion
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

    # Start yaw
    yaw = yaw0

    # Start time
    t = 0

    for i in range(200):
        t += DT

        plotter.ts.append(t)

        # Spin around while moving
        yaw = norm_angle(yaw + 0.05)

        plotter.yaws.append(yaw / pi * 180.)

        # Drag depends on yaw and direction of motion
        drag_coef_x, drag_coef_y = drag_coef_world(yaw, angle_to_goal)

        # Total acceleration is fixed
        plotter.axs.append(ax)
        plotter.ays.append(ay)

        # Acceleration due to drag
        drag_ax = drag_accel(vx, drag_coef_x)
        drag_ay = drag_accel(vy, drag_coef_y)

        plotter.daxs.append(drag_ax)
        plotter.days.append(drag_ay)

        # Acceleration due to thrust
        thrust_ax = ax - drag_ax
        thrust_ay = ay - drag_ay

        plotter.taxs.append(thrust_ax)
        plotter.tays.append(thrust_ay)

        # Thrust effort
        effort_x = accel_to_effort(thrust_ax)
        effort_y = accel_to_effort(thrust_ay)

        # Rotate thrust from world frame to body frame
        effort_forward, effort_strafe = rotate_frame(effort_x, effort_y, yaw)

        plotter.efs.append(effort_forward)
        plotter.ess.append(effort_strafe)

        # Velocity
        vx += ax * DT
        vy += ay * DT

        plotter.vxs.append(vx)
        plotter.vys.append(vy)

        # When we hit target velocity, set acceleration = 0
        # This should happen at the same time!
        if vx >= target_vx or vx <= -target_vx:
            ax = 0
        if vy >= target_vy or vy <= -target_vy:
            ay = 0

        # Position
        x += vx * DT
        y += vy * DT

        plotter.xs.append(x)
        plotter.ys.append(y)


def main():
    plotter = Plotter()

    constant_thrust(plotter, 0., 0., 10., 10., 0.)
    # constant_acceleration(plotter, 0., 0., 10., 10., 0.)

    plotter.plot()


if __name__ == '__main__':
    main()
