"""
A trajectory is a path sampled at a time-scaling. This module provides
convenience functions for commonly combined paths and time-scalings.

f(t) ≡ p(s(t))
f(t): [0, T] -> R^3
"""

from taskspace_control_examples.path import *
from taskspace_control_examples.time_scaling import *


def slerp_traj(q_start, q_end, tf, scaling=Order.FIRST):
    f, f_dot = slerp_path(q_start, q_end)
    s, s_dot = scaling(tf)
    return lambda t: f(s(t)), lambda t: f_dot(s(t), s_dot(t))


def linear_traj(p_start, p_end, tf, scaling=Order.FIFTH):
    f, f_dot = linear_path(p_start, p_end)
    s, s_dot = scaling(tf)
    return lambda t: f(s(t)), lambda t: f_dot(None, s_dot(t))


def circular_traj(radius, tf, phase=0.0, scaling=Order.FIFTH):
    f, f_dot = circle(radius, phase)
    s, s_dot = scaling(tf)
    return lambda t: f(s(t)), lambda t: f_dot(s(t), s_dot(t))


def hypotrochoid_traj(r, R, d, tf, phase=0.0, scaling=Order.FIFTH):
    f, f_dot = hypotrochoid(r, R, d, phase)
    s, s_dot = scaling(tf)
    return lambda t: f(s(t)), lambda t: f_dot(s(t), s_dot(t))


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    p_start = np.array([0.0, 0.0, 0.0])
    p_end = np.array([0.5, 0.75, 1.0])

    tf = 5
    tt = np.linspace(0, tf, 1000)

    # First order linear trajectory
    f, f_dot = linear_traj(p_start, p_end, tf, scaling=Order.FIRST)

    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, f(tt), label=["x", "y", "z"])
    ax[0].set_title("First-order time scaling: position")
    ax[0].legend(loc="upper right")
    ax[1].plot(tt, f_dot(tt), label=["x", "y", "z"])
    ax[1].set_title("First-order time scaling: velocity")
    ax[1].legend(loc="upper right")

    # Third order linear trajectory
    f, f_dot = linear_traj(p_start, p_end, tf, scaling=Order.THIRD)

    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, f(tt), label=["x", "y", "z"])
    ax[0].set_title("Third-order time scaling: position")
    ax[0].legend(loc="upper right")
    ax[1].plot(tt, f_dot(tt), label=["x", "y", "z"])
    ax[1].set_title("Third-order time scaling: velocity")
    ax[1].legend(loc="upper right")

    # Fifth order linear trajectory
    f, f_dot = linear_traj(p_start, p_end, tf, scaling=Order.FIFTH)

    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, f(tt), label=["x", "y", "z"])
    ax[0].set_title("Fifth-order time scaling: position")
    ax[0].legend(loc="upper right")
    ax[1].plot(tt, f_dot(tt), label=["x", "y", "z"])
    ax[1].set_title("Fifth-order time scaling: velocity")
    ax[1].legend(loc="upper right")

    # Circular Trajectory
    radius = 3
    f, f_dot = circular_traj(radius, tf, scaling=Order.FIFTH)
    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, f(tt), label=["x", "y", "z"])
    ax[0].set_title("Fifth-order time scaling: position")
    ax[0].legend(loc="upper right")
    ax[1].plot(tt, f_dot(tt), label=["x", "y", "z"])
    ax[1].set_title("Fifth-order time scaling: velocity")
    ax[1].legend(loc="upper right")

    # Hypotrochoid Trajectory
    scale = 3
    f, f_dot = hypotrochoid_traj(3, 5, 4.5, tf, scaling=Order.FIFTH)
    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, f(tt), label=["x", "y", "z"])
    ax[0].set_title("Fifth-order time scaling: position")
    ax[0].legend(loc="upper right")
    ax[1].plot(tt, f_dot(tt), label=["x", "y", "z"])
    ax[1].set_title("Fifth-order time scaling: velocity")
    ax[1].legend(loc="upper right")

    plt.tight_layout()
    plt.show()
