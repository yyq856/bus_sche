"""
SUE + MNL hyper-parameter store.

All non-decision parameters for the stochastic user equilibrium (route) and
multinomial logit (mode) lower-level are centralized here so you can tune them
without touching the model code.
"""
from __future__ import annotations

import math
from typing import Dict, Iterable, List, Tuple

# Dispersion parameters
THETA_DEFAULT = 1.0  # Path-choice dispersion (SUE)
MU_DEFAULT = 1.0     # Mode-choice dispersion (MNL)

# Big-M value for bilinear linearizations (design y * dual variable)
BIG_M_DEFAULT = 1e6

# Default linearization breakpoints
# - Beckmann integral: link flow breakpoints as multiples of capacity
# - Entropy terms: flows/demands breakpoints to outer-approximate x ln x
BECKMANN_FLOW_BREAKPOINTS = [0.0, 0.5, 1.0, 1.5]  # Reduced for faster testing
ENTROPY_PATH_BREAKPOINTS = [1e-3, 0.1, 1.0]  # Reduced for faster testing
ENTROPY_MODE_BREAKPOINTS = [1e-2, 0.1, 1.0]  # Reduced for faster testing


def get_sue_hyperparams(
    theta: float | None = None,
    mu: float | None = None,
    big_m: float | None = None,
    beckmann_bps: Iterable[float] | None = None,
    path_entropy_bps: Iterable[float] | None = None,
    mode_entropy_bps: Iterable[float] | None = None,
) -> Dict:
    """Return a mutable dict of hyper-parameters for the SUE+MNL model.

    Override any argument to quickly experiment with different settings.
    """
    return {
        "theta": float(theta) if theta is not None else THETA_DEFAULT,
        "mu": float(mu) if mu is not None else MU_DEFAULT,
        "big_m": float(big_m) if big_m is not None else BIG_M_DEFAULT,
        "beckmann_flow_breakpoints": list(beckmann_bps) if beckmann_bps is not None else list(BECKMANN_FLOW_BREAKPOINTS),
        "entropy_path_breakpoints": list(path_entropy_bps) if path_entropy_bps is not None else list(ENTROPY_PATH_BREAKPOINTS),
        "entropy_mode_breakpoints": list(mode_entropy_bps) if mode_entropy_bps is not None else list(ENTROPY_MODE_BREAKPOINTS),
    }


def build_entropy_tangents(breakpoints: Iterable[float]) -> List[Tuple[float, float]]:
    """
    Pre-compute (slope, intercept) pairs for outer-approximating x ln x at
    selected breakpoints. Each tangent: y >= (1 + ln bp) * x - bp.
    Returns list of (slope, intercept).
    """
    tangents: List[Tuple[float, float]] = []
    for bp in breakpoints:
        bp_val = float(bp)
        if bp_val <= 0:
            continue  # avoid log of non-positive
        slope = 1.0 + math.log(bp_val)
        intercept = -bp_val
        tangents.append((slope, intercept))
    return tangents


def build_beckmann_tangents(capacity: float, t0: float, alpha: float, beta: float, breakpoints: Iterable[float]) -> List[Tuple[float, float]]:
    """
    Generate tangents (slope, intercept) for the Beckmann integral outer
    approximation at specified flow ratios (v/C). t0 in hours, capacity in pcu/hr.
    Returns list of (slope, intercept) where slope = t(v_bp) and intercept = B(v_bp) - t(v_bp)*v_bp.
    """
    tangents: List[Tuple[float, float]] = []
    for ratio in breakpoints:
        v_bp = float(ratio) * capacity
        if v_bp < 0:
            continue
        if capacity > 0:
            t_bp = t0 * (1 + alpha * (v_bp / capacity) ** beta)
            beckmann_bp = t0 * (v_bp + alpha * capacity / (beta + 1) * (v_bp / capacity) ** (beta + 1))
        else:
            t_bp = t0
            beckmann_bp = t0 * v_bp
        intercept = beckmann_bp - t_bp * v_bp
        tangents.append((t_bp, intercept))
    return tangents


__all__ = [
    "get_sue_hyperparams",
    "build_entropy_tangents",
    "build_beckmann_tangents",
    "THETA_DEFAULT",
    "MU_DEFAULT",
    "BIG_M_DEFAULT",
    "BECKMANN_FLOW_BREAKPOINTS",
    "ENTROPY_PATH_BREAKPOINTS",
    "ENTROPY_MODE_BREAKPOINTS",
]
