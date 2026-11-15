"""Rigid-body equations of motion.

Reproduces the Aerospace Blockset '6DOF (Quaternion)' block as instantiated in
AE4803Sim (SID 328): body axes, Metric (MKS), fixed mass, quaternion
representation, k_quat = 1.0.

Quaternion convention (Aerospace Blockset): q = [q0, q1, q2, q3] with q0 the
scalar part, representing the rotation from EARTH to BODY axes, so that
DCM_be = dcm_from_quat(q) maps earth-frame vectors into body axes.

State vector (13 elements), in this order:
    0:3   xe    flat-earth position  [m]   (NED: z positive down)
    3:6   Vb    body-axis velocity   [m/s]
    6:10  q     quaternion (earth -> body)
    10:13 omega body angular rates   [rad/s]
"""
from __future__ import annotations

import numpy as np

IDX_XE = slice(0, 3)
IDX_VB = slice(3, 6)
IDX_Q = slice(6, 10)
IDX_W = slice(10, 13)
N_STATES = 13


def dcm_from_quat(q: np.ndarray) -> np.ndarray:
    """DCM mapping earth-frame vectors to body axes."""
    q0, q1, q2, q3 = q
    return np.array([
        [q0 * q0 + q1 * q1 - q2 * q2 - q3 * q3,
         2.0 * (q1 * q2 + q0 * q3),
         2.0 * (q1 * q3 - q0 * q2)],
        [2.0 * (q1 * q2 - q0 * q3),
         q0 * q0 - q1 * q1 + q2 * q2 - q3 * q3,
         2.0 * (q2 * q3 + q0 * q1)],
        [2.0 * (q1 * q3 + q0 * q2),
         2.0 * (q2 * q3 - q0 * q1),
         q0 * q0 - q1 * q1 - q2 * q2 + q3 * q3],
    ])


def euler_from_dcm(dcm: np.ndarray) -> np.ndarray:
    """[roll, pitch, yaw] in radians from DCM_be (3-2-1 sequence)."""
    phi = np.arctan2(dcm[1, 2], dcm[2, 2])
    theta = -np.arcsin(np.clip(dcm[0, 2], -1.0, 1.0))
    psi = np.arctan2(dcm[0, 1], dcm[0, 0])
    return np.array([phi, theta, psi])


def quat_from_euler(eul: np.ndarray) -> np.ndarray:
    """Quaternion (earth -> body) from [roll, pitch, yaw] in radians."""
    phi, theta, psi = (float(x) for x in eul)
    cp, sp = np.cos(phi / 2.0), np.sin(phi / 2.0)
    ct, st = np.cos(theta / 2.0), np.sin(theta / 2.0)
    cs, ss = np.cos(psi / 2.0), np.sin(psi / 2.0)
    return np.array([
        cp * ct * cs + sp * st * ss,
        sp * ct * cs - cp * st * ss,
        cp * st * cs + sp * ct * ss,
        cp * ct * ss - sp * st * cs,
    ])


def quat_rate(q: np.ndarray, omega: np.ndarray, k_quat: float = 1.0) -> np.ndarray:
    """Quaternion derivative with the block's normalisation feedback term.

    qdot = -1/2 * [[ 0,  p,  q,  r],
                   [-p,  0, -r,  q],
                   [-q,  r,  0, -p],
                   [-r, -q,  p,  0]] q   +  k_quat * (1 - |q|^2) * q
    """
    q0, q1, q2, q3 = q
    p, qq, r = float(omega[0]), float(omega[1]), float(omega[2])
    qdot = 0.5 * np.array([
        -(p * q1 + qq * q2 + r * q3),
        p * q0 + r * q2 - qq * q3,
        qq * q0 - r * q1 + p * q3,
        r * q0 + qq * q1 - p * q2,
    ])
    eps = 1.0 - float(np.dot(q, q))
    return qdot + k_quat * eps * q


def state_derivative(state: np.ndarray, force_b: np.ndarray, moment_b: np.ndarray,
                     mass: float, inertia: np.ndarray, inertia_inv: np.ndarray,
                     k_quat: float = 1.0) -> np.ndarray:
    """Full 13-state derivative.

        dxe/dt    = DCM_be^T * Vb
        dVb/dt    = F/m - omega x Vb
        dq/dt     = quat_rate(q, omega)
        domega/dt = I^-1 (M - omega x (I omega))
    """
    vb = state[IDX_VB]
    q = state[IDX_Q]
    w = state[IDX_W]

    dcm = dcm_from_quat(q)
    ve = dcm.T @ vb

    dvb = force_b / mass - np.cross(w, vb)
    dw = inertia_inv @ (moment_b - np.cross(w, inertia @ w))
    dq = quat_rate(q, w, k_quat)

    out = np.empty(N_STATES)
    out[IDX_XE] = ve
    out[IDX_VB] = dvb
    out[IDX_Q] = dq
    out[IDX_W] = dw
    return out


def make_initial_state(xme_0: np.ndarray, uvw0: np.ndarray,
                       euler0: np.ndarray, pqr0: np.ndarray) -> np.ndarray:
    s = np.zeros(N_STATES)
    s[IDX_XE] = np.asarray(xme_0, dtype=float)
    s[IDX_VB] = np.asarray(uvw0, dtype=float)
    s[IDX_Q] = quat_from_euler(np.asarray(euler0, dtype=float))
    s[IDX_W] = np.asarray(pqr0, dtype=float)
    return s


def rk4_step(f, t: float, y: np.ndarray, h: float) -> np.ndarray:
    """Classical fixed-step RK4. ``f(t, y) -> dy/dt``."""
    k1 = f(t, y)
    k2 = f(t + 0.5 * h, y + 0.5 * h * k1)
    k3 = f(t + 0.5 * h, y + 0.5 * h * k2)
    k4 = f(t + h, y + h * k3)
    return y + (h / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4)
