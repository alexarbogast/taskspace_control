# Copyright 2024 Alex Arbogast
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import numpy as np

_EPS = 1e-10


def quaternion_normalize(q):
    """Normalize quaternion(s) to unit length."""
    norm = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / norm


def quaternion_conjugate(q):
    """Return the conjugate of quaternion(s)."""
    w = q[..., :1]
    xyz = q[..., 1:]
    return np.concatenate([w, -xyz], axis=-1)


def quaternion_inv(q):
    """Return the inverse of quaternion(s)."""
    return quaternion_conjugate(q) / np.sum(q * q, axis=-1, keepdims=True)


def quaternion_multiply(q1, q2):
    """Multiply two quaternions: q = q1 ⊗ q2 (apply q2, then q1)"""
    w1, x1, y1, z1 = q1[..., 0], q1[..., 1], q1[..., 2], q1[..., 3]
    w2, x2, y2, z2 = q2[..., 0], q2[..., 1], q2[..., 2], q2[..., 3]

    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2
    z = w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2

    return np.stack([w, x, y, z], axis=-1)


def quaternion_log(q):
    """Compute the logarithm of a unit quaternion."""
    q = quaternion_normalize(q)

    w = q[..., 0]
    v = q[..., 1:]

    v_norm = np.linalg.norm(v, axis=-1, keepdims=True)

    # Avoid division by zero
    theta = np.arctan2(v_norm[..., 0], w)
    result = np.zeros_like(q)

    # When v_norm is very small, log(q) ≈ [0, 0, 0, 0]
    mask = v_norm[..., 0] > _EPS

    if np.any(mask):
        scale = theta[mask] / v_norm[mask, 0]
        result[mask, 1:] = v[mask] * scale[..., np.newaxis]

    return result


def quaternion_exp(q):
    """Compute the quaternion exponential"""
    w = q[..., :1]
    v = q[..., 1:]

    alpha = np.linalg.norm(v, axis=-1, keepdims=True)

    scale = np.empty_like(alpha)
    small = alpha < _EPS

    scale[small] = 1.0
    scale[~small] = np.sin(alpha[~small]) / alpha[~small]

    exp_w = np.exp(w)
    return np.concatenate([exp_w * np.cos(alpha), exp_w * scale * v], axis=-1)


def quaternion_from_rotation_vec(vec):
    """Compute the quaternion from a (θ * v)"""
    vec = np.asarray(vec, dtype=float)

    q = np.zeros(vec.shape[:-1] + (4,))
    q[..., 1:] = 0.5 * vec

    return quaternion_exp(q)
