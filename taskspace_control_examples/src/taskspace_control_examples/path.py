"""
A path maps a sampling parameter "s" to a vector in R^n.
A path sampled at a time-scaling s(t) gives a trajectory.

p(s): [0, 1] -> R^n
p(s(t)): [0, T] -> R^n
"""

import numpy as np


def array_function(func):
    def wrapper(*args):
        args = [np.atleast_1d(a)[:, np.newaxis] for a in args]
        result = func(*args)
        return np.squeeze(result)

    return wrapper


def linear_path(p_start, p_end):
    @array_function
    def f(s):
        fs = p_start + (p_end - p_start) * s
        return fs

    @array_function
    def f_dot(s, s_dot):
        fdot_sdot = (p_end - p_start) * s_dot
        return fdot_sdot

    return f, f_dot


def circle(radius, phase=0.0):
    def f(s):
        s2pi = s * 2 * np.pi
        coss2pi = np.cos(s2pi + phase)
        sins2pi = np.sin(s2pi + phase)
        fs = radius * np.array([coss2pi, sins2pi, np.zeros_like(s)]).T
        return fs

    @array_function
    def f_dot(s, s_dot):
        s2pi = s * 2 * np.pi
        sdot2pi = s_dot * 2 * np.pi
        coss2pi = np.cos(s2pi + phase)
        sins2pi = np.sin(s2pi + phase)
        fdot_sdot = radius * sdot2pi * np.array([-sins2pi, coss2pi, np.zeros_like(s)]).T
        return fdot_sdot

    return f, f_dot


def hypotrochoid(r, R, d, phase=0.0):
    # https://www.desmos.com/calculator/pvp5n6kjxt
    Rmr = R - r
    ω = (r * R) / R * 2 * np.pi
    φ = phase

    def f(s):
        cosf1, cosf2 = np.cos(ω * s + φ), np.cos(Rmr / r * (ω * s + φ))
        sinf1, sinf2 = np.sin(ω * s + φ), np.sin(Rmr / r * (ω * s + φ))
        fs = np.array(
            [
                Rmr * cosf1 + d * cosf2,
                Rmr * sinf1 - d * sinf2,
                np.zeros_like(s),
            ]
        ).T
        return fs

    def f_dot(s, s_dot):
        cosf1, cosf2 = np.cos(ω * s + φ), np.cos(Rmr / r * (ω * s + φ))
        sinf1, sinf2 = np.sin(ω * s + φ), np.sin(Rmr / r * (ω * s + φ))
        fdot_sdot = np.array(
            [
                -s_dot * Rmr * ω * (sinf1 + (d / r * sinf2)),
                s_dot * Rmr * ω * (cosf1 - (d / r * cosf2)),
                np.zeros_like(s),
            ]
        ).T
        return fdot_sdot

    return f, f_dot


if __name__ == "__main__":
    import matplotlib.pyplot as plt

    s = np.linspace(0, 1, 100)

    # linear path
    p_start = np.array([0.0, 0.0, 0.0])
    p_end = np.array([0.5, 0.75, 1.0])

    f, f_dot = linear_path(p_start, p_end)
    fs = f(s)

    plt.figure()
    plt.plot(fs[:, 0], fs[:, 1])
    plt.title("Linear Path")

    # circle
    f, f_dot = circle(5)
    fs = f(s)

    plt.figure()
    plt.plot(fs[:, 0], fs[:, 1])
    plt.title("Circular Path")

    plt.tight_layout()

    # hypotrochoid
    f, f_dot = hypotrochoid(3, 5, 4.5)
    fs = f(s)

    plt.figure()
    plt.plot(fs[:, 0], fs[:, 1])
    plt.title("Hypotrochoid Path")

    plt.tight_layout()
    plt.show()
