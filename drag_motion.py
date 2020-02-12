#!/usr/bin/env python3

"""
Analyze and plot 2d motion with drag

Run in a straight line from (x0, y0) to (x1, y1) with velocity target_v
Yaw changes from yaw0 to yaw1

How do I control thrust effort (forward, strafe, yaw) to achieve this?
"""

from math import cos, sin, atan2, pi, hypot, sqrt

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

XY_TARGET_V = 1.  # Target xy velocity in m/s
YAW_TARGET_V = 0.5  # Target yaw velocity in r/s
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

    target_vx = cos(angle_to_goal) * XY_TARGET_V
    target_vy = sin(angle_to_goal) * XY_TARGET_V
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


XY_A = 0.25  # Acceleration rate in m / s^2
YAW_A = 1.  # Acceleration rate in r / s^2


def plan(acceleration, max_velocity, distance):
    """
    Plan a 3-phase motion that runs a distance constrained by acceleration and max_velocity:
        phase 1: accelerate
        phase 2: run at constant velocity
        phase 3: decelerate and stop

    Return 3 times:
        (start_ramp_up_t is always 0)
        start_run_t
        start_ramp_down_t
        stop_t

    If the distance is short, start_run_t == start_ramp_down_t (run phase is skipped)
    """

    ramp_duration = max_velocity / acceleration
    ramp_distance = 1 / 2 * acceleration * ramp_duration * ramp_duration

    if 2 * ramp_distance < distance:
        # Will hit target_velocity
        run_duration = (distance - 2 * ramp_distance) / max_velocity
    else:
        # Will not hit target_velocity
        ramp_duration = sqrt(distance / acceleration)
        run_duration = 0

    print('ramp_duration', ramp_duration, 'run_duration', run_duration)

    start_run_t = ramp_duration
    start_ramp_down_t = start_run_t + run_duration
    stop_t = start_ramp_down_t + ramp_duration

    return start_run_t, start_ramp_down_t, stop_t


def constant_velocity(plotter: Plotter, x0, y0, x1, y1, yaw0, yaw1):
    """
    Constant velocity (with ramp up / down), works well for any drag configuration

    Run yaw and xy motion at the same time
    The loop stops when both motions are complete

    Future: slow down the fastest motion so that all motions complete at the same time
    """

    # Plan yaw motion
    yaw_start_run_t, yaw_start_ramp_down_t, yaw_stop_t = plan(YAW_A, YAW_TARGET_V, norm_angle(yaw1 - yaw0))

    # Start yaw acceleration
    ayaw = YAW_A

    # Start yaw velocity
    vyaw = 0.

    # Start yaw
    yaw = yaw0

    # Plan xy motion
    xy_distance = hypot(x1 - x0, y1 - y0)
    xy_start_run_t, xy_start_ramp_down_t, xy_stop_t = plan(XY_A, XY_TARGET_V, xy_distance)

    # Split xy velocity into x and y components
    angle_to_goal = atan2(y1 - y0, x1 - x0)
    print('angle to goal', angle_to_goal)
    target_vx = cos(angle_to_goal) * XY_TARGET_V
    target_vy = sin(angle_to_goal) * XY_TARGET_V
    print('target_vx', target_vx)
    print('target_vy', target_vy)

    # Start x and y acceleration
    if xy_distance > 0:
        ax = target_vx / xy_start_run_t
        ay = target_vy / xy_start_run_t
    else:
        ax = 0
        ay = 0

    # Start x and y velocity
    vx = 0.
    vy = 0.

    # Start x and y
    x = x0
    y = y0

    # Start time
    t = 0

    while t < yaw_stop_t or t < xy_stop_t:

        ###################
        # Time
        ###################

        t += DT

        plotter.ts.append(t)

        ###################
        # Yaw motion
        # Future: deal with yaw drag
        ###################

        # Velocity
        vyaw += ayaw * DT

        # Position
        yaw = norm_angle(yaw + vyaw * DT)

        plotter.yaws.append(yaw / pi * 180.)

        if t > yaw_stop_t:
            # Done
            ayaw = 0
            vyaw = 0
        elif t > yaw_start_ramp_down_t:
            # Enter velocity ramp down phase
            ayaw = -YAW_TARGET_V / yaw_start_run_t
        elif t > yaw_start_run_t:
            # Enter run phase
            ayaw = 0

        ###################
        # X and Y motion
        ###################

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

        # Position
        x += vx * DT
        y += vy * DT

        plotter.xs.append(x)
        plotter.ys.append(y)

        if t > xy_stop_t:
            # Done
            ax = 0
            ay = 0
            vx = 0
            vy = 0
        elif t > xy_start_ramp_down_t:
            # Enter velocity ramp down phase
            ax = -target_vx / xy_start_run_t
            ay = -target_vy / xy_start_run_t
        elif t > xy_start_run_t:
            # Enter run phase
            ax = 0
            ay = 0


def main():
    plotter = Plotter()

    # constant_thrust(plotter, 0., 0., 10., 10., 0.)
    
    constant_velocity(plotter, 0., 0., 10., 10., 0., pi)  # x, y and yaw
    # constant_velocity(plotter, 0., 0., 10., 10., 0., 0.)  # x and y
    # constant_velocity(plotter, 0., 0., 10., 0., 0., pi)  # x and yaw
    # constant_velocity(plotter, 0., 0., 0., 10., 0., pi)  # y and yaw
    # constant_velocity(plotter, 0., 0., 10., 0., 0., 0.)  # Just x
    # constant_velocity(plotter, 0., 0., 0., 10., 0., 0.)  # Just y
    # constant_velocity(plotter, 0., 0., 0., 0., 0., pi)  # Just yaw
    # constant_velocity(plotter, 0., 0., 0., 0., 0., 0.)  # Nothing

    plotter.plot()


if __name__ == '__main__':
    main()
