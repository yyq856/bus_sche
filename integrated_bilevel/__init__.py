#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Integrated Bi-Level Model: Upper-level System Cost + Lower-level SUE-MNL via Strong Duality

This package implements the fusion of:
  - Upper Level (Leader): System cost minimization (operator + user + background costs)
  - Lower Level (Follower): Stochastic User Equilibrium (SUE) with Multinomial Logit (MNL) mode choice

The bi-level problem is reformulated into a single-level MILP via:
  1. Piecewise linear approximation (outer approximation) of convex terms
  2. Strong duality theorem to replace lower-level optimization with constraints
  3. Big-M linearization of bilinear terms

Reference: See model_formulation.tex for mathematical details.
"""

__version__ = "1.0.0"
__author__ = "Transportation Optimization Team"

from .model import build_integrated_bilevel_model
from .solve import solve_bilevel_problem
from .parameters import (
    get_sue_hyperparams,
    get_breakpoints,
    get_cost_params,
    get_utility_params,
    get_all_params,
)

__all__ = [
    "build_integrated_bilevel_model",
    "solve_bilevel_problem",
    "get_sue_hyperparams",
    "get_breakpoints",
    "get_cost_params",
    "get_utility_params",
    "get_all_params",
]
