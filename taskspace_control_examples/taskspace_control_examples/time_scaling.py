"""
A polynomial time-scaling converts a time "t" to a parameter in the range of 0 to 1.
A path sampled at a time-scaling s(t) gives a trajectory.

s(t): [0, T] -> [0, 1]
"""

import numpy as np
from enum import Enum


def first_order_scaling(tf):
    s = np.polynomial.Polynomial([0.0, 1 / tf])
    s_dot = s.deriv()
    return s, s_dot


def third_order_scaling(tf):
    s = np.polynomial.Polynomial([0.0, 0.0, 3 / tf**2, -2 / tf**3])
    s_dot = s.deriv()
    return s, s_dot


def fifth_order_scaling(tf):
    s = np.polynomial.Polynomial([0.0, 0.0, 0.0, 10 / tf**3, -15 / tf**4, 6 / tf**5])
    s_dot = s.deriv()
    return s, s_dot


class Order(Enum):
    FIRST = first_order_scaling
    THIRD = third_order_scaling
    FIFTH = fifth_order_scaling


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    tf = 5
    tt = np.linspace(0, tf, 1000)

    # First-order time parameterization
    s, s_dot = first_order_scaling(tf)

    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, s(tt))
    ax[0].set_title("First-order time parameterization (s)")
    ax[1].plot(tt, s_dot(tt))
    ax[1].set_title("First-order time parameterization (s_dot)")

    # Third-order time parameterization
    s, s_dot = third_order_scaling(tf)

    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, s(tt))
    ax[0].set_title("Third-order time parameterization (s)")
    ax[1].plot(tt, s_dot(tt))
    ax[1].set_title("Third-order time parameterization (s_dot)")

    # Fifth-order time parameterization
    s, s_dot = fifth_order_scaling(tf)

    _, ax = plt.subplots(2, 1)
    ax[0].plot(tt, s(tt))
    ax[0].set_title("Fifth-order time parameterization (s)")
    ax[1].plot(tt, s_dot(tt))
    ax[1].set_title("Fifth-order time parameterization (s_dot)")

    plt.show()
