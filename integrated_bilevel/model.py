#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Integrated Bi-Level Model: Core model construction.

Implements:
  1. Upper-level system cost objective (from code/)
  2. Lower-level SUE-MNL via strong duality (from code_sue/)
  3. Unified variable and constraint blocks
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List, Tuple, Optional

import networkx as nx

# Import Gurobi wrapper
try:
    from .gurobi_wrapper import GurobiModelWrapper as Model
except ImportError:
    from gurobi_wrapper import GurobiModelWrapper as Model

# Add parent path for code/ imports
code_path = str(Path(__file__).resolve().parent.parent / "code")
if code_path not in sys.path:
    sys.path.insert(0, code_path)

# Local imports from integrated_bilevel/
# Try relative import first, fall back to absolute
try:
    from .utils import (
        compute_path_time_hr,
        build_initial_bus_paths,
        build_path_dict,
        build_link_path_incidence,
        get_fixed_utility,
    )
    from .parameters import (
        BPR_ALPHA,
        BPR_BETA,
        BETA_TT,
        build_beckmann_tangents,
        build_entropy_tangents,
        get_sue_hyperparams,
    )
except ImportError:
    from utils import (
        compute_path_time_hr,
        build_initial_bus_paths,
        build_path_dict,
        build_link_path_incidence,
        get_fixed_utility,
    )
    from parameters import (
        BPR_ALPHA,
        BPR_BETA,
        BETA_TT,
        build_beckmann_tangents,
        build_entropy_tangents,
        get_sue_hyperparams,
    )

PathId = Tuple[int, int, str, int]  # (o, d, mode, path_idx)


def build_integrated_bilevel_model(
    params: Dict,
    hyper: Dict,
    verbose: bool = True,
    P_hat: Optional[Dict] = None,
    relax_to_lp: bool = False,
) -> Dict:
    """
    Build the integrated bi-level MILP model via strong duality reformulation.
    
    Can also build LP relaxation by setting relax_to_lp=True.
    
    CRITICAL MODEL STRUCTURE (Fleet-Based Optimization):
    ====================================================
    
    UPPER LEVEL DECISION VARIABLES:
      - x_r ∈ {0,1}: Route activation (operate this route or not)
      - w_rk ∈ {0,1}: Headway selection (which frequency class for this route)
      - n_r ∈ ℤ₊: FLEET SIZE (number of buses for this route) -- EXPLICIT DECISION VARIABLE
    
    UPPER LEVEL OBJECTIVE:
      - Old model: min Z_op(x,w) = FC_r·x_r + C_op_rk·w_rk + ...
      - NEW model: min Z_op(x,n) = FC_r·x_r + C_fleet·n_r + ...
      
    KEY INSIGHT: Operating cost is NOW TIED TO FLEET SIZE (n_r), NOT FREQUENCY (w_rk)
    
    This reflects real transit operations where:
      1. Bus purchase/maintenance costs depend on NUMBER OF VEHICLES (n_r)
      2. Frequency selection (w_rk) affects service quality, but does NOT directly determine fleet cost
      3. The constraints ensure n_r is SUFFICIENT for the chosen frequency:
         - n_r·C_B ≥ (total passenger demand on route r)
         - n_r·H_j ≥ (cycle_time_hr[r]·w_rj) for selected headway j
    
    CONSTRAINTS (Upper Level):
      1. Frequency selection: Σ_k w_rk = x_r (exactly one frequency when route is active)
      2. Fleet capacity: n_r·C_B ≥ total flow on route r (seated capacity constraint)
      3. Fleet activation: n_r ≤ M·x_r (no buses for inactive routes)
      4. Headway feasibility: n_r·H_j ≥ cycle_time_r·w_rj (enough buses for frequency)
    
    LOWER LEVEL (User Equilibrium):
      - SUE+MNL route/mode choice in response to (x,w,n) decisions
      - Returns: f (path flows), q (mode demands), v (link flows)
      - Strong duality reformulation replaces lower-level optimization
    
    Args:
        params: Model parameters (from code/ pipeline)
        hyper: Hyperparameters (theta, mu, breakpoints)
        verbose: Print status messages
        P_hat: Optional dict of bus routes {route_id: {nodes, od_pair, route_id}}
               If None, will generate initial shortest paths
        relax_to_lp: If True, build LP relaxation (continuous vars, no KKT duals)
        
    Returns:
        Dict with keys:
          - "model": Docplex Model object
          - "variables": Dict of all variable collections (includes n)
          - "obj_components": Dict of objective function terms
          - "path_dict": Unified path dictionary
          - "P_hat": Bus paths
          - "cycle_time_hr": Cycle time for each route (for reference)
    """
    G = params["graph"]
    A = params["A"]
    K = params["K"]
    H = params["H"]
    M_h = params["M_h"]
    D_odh = params["D_odh"]
    
    # Extract all modes from M_h (set of all modes available to any segment)
    M = list(set(m for modes in M_h.values() for m in modes))
    # Ensure standard mode ordering for consistency: D, X, B, R, W, O
    standard_order = ['D', 'X', 'B', 'R', 'W', 'O']
    M = [m for m in standard_order if m in M] + [m for m in M if m not in standard_order]

    # ===== RULE 1 FIX: Mode Classification =====
    # Separate modes into Fixed-demand (Auto) and Choice modes (elastic demand via MNL)
    # Auto/D: Fixed demand, SUE path choice only, NO MNL entropy
    # Other modes (X, B, R, W, O): Elastic demand, MNL allocation + SUE path choice
    M_Fixed = {'D'}  # Captive auto drivers
    M_Choice = set(M) - M_Fixed  # Elastic demand modes
    
    # Auto demand share: percentage of total demand that goes to auto (fixed)
    # This split is exogenous; auto drivers don't participate in mode choice
    auto_demand_share = params.get('auto_demand_share', 0.3)  # Default 30% auto share

    mdl = Model(name="Integrated_Bilevel_SUE_MNL")

    # --- Phase 1: Data preparation ---
    if verbose:
        print("[Phase 1] Preparing data structures...")
    
    # Use provided bus routes or generate initial ones
    if P_hat is None:
        P_hat = build_initial_bus_paths(params)
    elif 'P_hat' in params:
        # If P_hat is in params, use it
        P_hat = params['P_hat']
    
    path_dict = build_path_dict(params, P_hat)
    incidence = build_link_path_incidence(path_dict)
    
    if verbose:
        print(f"  - Bus paths: {len(P_hat)}")
        print(f"  - Total paths (all modes): {len(path_dict)}")

    # --- Phase 2: Hyperparameters & tangents ---
    if verbose:
        print("[Phase 2] Building approximation tangents...")
    
    theta = float(hyper["theta"])
    mu = float(hyper["mu"])
    beck_bps = hyper["beckmann_flow_breakpoints"]
    path_tangents = build_entropy_tangents(hyper["entropy_path_breakpoints"])
    mode_tangents = build_entropy_tangents(hyper["entropy_mode_breakpoints"])

    # Pre-compute Beckmann tangents for all edges
    beck_tangents_by_edge = build_beckmann_tangents(G, beck_bps)
    
    if verbose:
        print(f"  - Beckmann breakpoints: {len(beck_bps)}")
        print(f"  - Path entropy tangents: {len(path_tangents)}")
        print(f"  - Mode entropy tangents: {len(mode_tangents)}")

    # --- Phase 3: Upper-level variables (system design) ---
    if verbose:
        print("[Phase 3] Creating upper-level variables...")
    
    r_indices = list(P_hat.keys())
    k_indices = list(params.get("J_r", {}).keys()) if "J_r" in params else \
                list(params.get("H_k", {}).keys()) if "H_k" in params else []
    headway_dict = params.get("H_k", params.get("J_r", {}))

    # --- Compute cycle time for each route (in hours) ---
    cycle_time_hr = {}
    for route_id, route_info in P_hat.items():
        path_nodes = route_info.get("nodes", [])
        # Use compute_path_time_hr to get travel time
        travel_time = compute_path_time_hr(G, path_nodes) if path_nodes else 0.0
        # Add dwell/buffer time (e.g., 5 minutes = 1/12 hour)
        dwell_time_hr = params.get("bus_dwell_time_hr", 1/12)  # default 5 minutes
        cycle_time_hr[route_id] = travel_time + dwell_time_hr

    # MODIFICATION FOR LP RELAXATION:
    # If relax_to_lp=True, create all variables as continuous [0,1]
    # Otherwise, use binary/integer as specified
    if relax_to_lp:
        # LP relaxation: all variables continuous
        x = mdl.continuous_var_dict(r_indices, lb=0, ub=1, name="x")  # Route activation
        n = mdl.continuous_var_dict(r_indices, lb=0, name="n")  # Fleet size
        w = {}
        if k_indices:
            w = mdl.continuous_var_dict(
                [(r, k) for r in r_indices for k in k_indices],
                lb=0, ub=1, name="w"
            )  # Headway selection
    elif hasattr(mdl, "binary_var_dict"):
        # Original MILP: use binary/integer
        x = mdl.binary_var_dict(r_indices, name="x")  # Route activation
        n = mdl.integer_var_dict(r_indices, lb=0, name="n")  # Fleet size
        w = {}
        if k_indices:
            w = mdl.binary_var_dict(
                [(r, k) for r in r_indices for k in k_indices],
                name="w"
            )  # Headway selection
    else:
        # Fallback to continuous (for environments without binary_var_dict)
        x = mdl.continuous_var_dict(r_indices, lb=0, ub=1, name="x")
        n = mdl.continuous_var_dict(r_indices, lb=0, name="n")
        w = {}
        if k_indices:
            w = mdl.continuous_var_dict(
                [(r, k) for r in r_indices for k in k_indices],
                lb=0, ub=1, name="w"
            )
    
    if verbose:
        print(f"  - Route activation variables (x): {len(x)}")
        print(f"  - Headway selection variables (w): {len(w)}")
        print(f"  - Fleet size variables (n): {len(n)}")
        if relax_to_lp:
            print(f"    (LP RELAXATION: all variables are continuous)")

    # --- Phase 4: Lower-level primal variables ---
    if verbose:
        print("[Phase 4] Creating lower-level primal variables...")
    
    v = mdl.continuous_var_dict(A, lb=0, name="v")  # Link flows (total)
    # Mode-specific link flows: v_am[a][m] = flow of mode m on link a
    v_am = {
        a: mdl.continuous_var_dict(
            M,  # All modes
            lb=0,
            name=f"v_m_{a}"
        )
        for a in A
    }
    f = mdl.continuous_var_dict(path_dict.keys(), lb=0, name="f")  # Path flows
    
    # ===== RULE 1 FIX: Mode demand variables =====
    # q_keys now includes ONLY Choice modes (B, R, X, W, O)
    # Auto (D) demand is NOT included here - it's handled as fixed demand
    # This ensures Auto does NOT get MNL entropy
    q_keys = [
        (o, d, h, m) for (o, d) in K for h in H for m in M_h[h] if m in M_Choice
    ]
    q = mdl.continuous_var_dict(q_keys, lb=0, name="q")  # Mode demands (Choice modes ONLY)

    # --- Mode-specific Beckmann approximation variables ---
    # NOTE: Per-link auxiliary variable (unified across all modes)
    # tau[a] approximates the global Beckmann integral: ∫[0, v_a] (∑_m β_time^m) t_a(ξ) dξ
    # This is weighted by the sum of mode-specific time coefficients
    tau = mdl.continuous_var_dict(A, lb=0, name="tau")  # Link Beckmann approximation (unified)
    
    phi = mdl.continuous_var_dict(path_dict.keys(), lb=0, name="phi")  # Path entropy approx
    
    # ===== RULE 1 FIX: Mode entropy variable =====
    # xi is ONLY defined for Choice modes
    # Auto (D) will NOT have an entropy variable, so no MNL weight applies to it
    xi = mdl.continuous_var_dict(q_keys, lb=0, name="xi")  # Mode entropy approx (Choice modes ONLY)
    
    if verbose:
        print(f"  - Link flow variables (v): {len(A)}")
        print(f"  - Path flow variables (f): {len(path_dict)}")
        print(f"  - Mode demand variables (q): {len(q_keys)}")

    # --- Phase 5: Lower-level dual variables ---
    if verbose:
        print("[Phase 5] Creating lower-level dual variables...")
    
    rho = mdl.continuous_var_dict(A, lb=None, ub=None, name="rho")  # Link dual
    
    # ===== RULE 1 FIX: lambda_wm needs ALL modes (including Auto) =====
    # Even though Auto doesn't have q variable, its paths still need lambda dual variable
    # Lambda represents the marginal utility of adding one unit of demand for mode m on OD (o,d)
    # For Auto: lambda connects path flows to fixed demand
    # For Choice: lambda connects path flows to elastic demand (q)
    # 
    # Extract all unique (o,d,m) combinations from path_dict
    # Convert to int() to avoid numpy type issues
    lambda_wm_keys = list(set(
        (int(pid[0]), int(pid[1]), pid[2]) for pid in path_dict.keys()
    ))
    lambda_wm = mdl.continuous_var_dict(lambda_wm_keys, lb=None, ub=None, name="lambda")  # Path-mode dual
    gamma_w = mdl.continuous_var_dict(K, lb=None, ub=None, name="gamma")  # OD demand dual

    # Beckmann dual cuts: ONLY indexed by (a, r), NOT mode-specific
    # This reflects the unified Beckmann integral with mode-weighted beta_time
    alpha_ar = {
        a: mdl.continuous_var_dict(
            range(len(beck_tangents_by_edge[a])),
            lb=0,
            name=f"alpha_{a}"
        )
        for a in A
    }
    
    beta_kj = mdl.continuous_var_dict(
        [(pid, j) for pid in path_dict for j in range(len(path_tangents))],
        lb=0,
        name="beta"
    )  # Path entropy dual cuts
    
    eta_wmj = mdl.continuous_var_dict(
        [(o, d, h, m, j) for (o, d, h, m) in q_keys for j in range(len(mode_tangents))],
        lb=0,
        name="eta"
    )  # Mode entropy dual cuts

    if verbose:
        print(f"  - Dual link variables (rho): {len(A)}")
        print(f"  - Dual path-mode variables (lambda): {len(lambda_wm)}")
        print(f"  - Dual OD variables (gamma): {len(K)}")

    # --- Phase 5b: McCormick auxiliary variables for bilinear terms ---
    if verbose:
        print("[Phase 5b] Creating McCormick auxiliary variables...")
    
    # For auto modes: zeta_f_time = f * travel_time
    # We create variables for paths that have auto or rail modes
    zeta_f_time_keys = [pid for pid in path_dict.keys() if pid[2] in ["D", "X", "R"]]
    zeta_f_time = mdl.continuous_var_dict(zeta_f_time_keys, lb=0, name="zeta_f_time")
    
    # For bus paths: zeta_f_TT = f * in_vehicle_time and zeta_f_WT = f * waiting_time
    zeta_f_TT_keys = [pid for pid in path_dict.keys() if pid[2] == "B"]
    zeta_f_TT = mdl.continuous_var_dict(zeta_f_TT_keys, lb=0, name="zeta_f_TT")
    zeta_f_WT = mdl.continuous_var_dict(zeta_f_TT_keys, lb=0, name="zeta_f_WT")

    # Piecewise-linear travel time for background cost (uses same breakpoints as Beckmann)
    t_bg = mdl.continuous_var_dict(A, lb=0, name="t_bg")
    
    if verbose:
        print(f"  - McCormick variables (zeta_f_time): {len(zeta_f_time)}")
        print(f"  - McCormick variables (zeta_f_TT): {len(zeta_f_TT)}")
        print(f"  - McCormick variables (zeta_f_WT): {len(zeta_f_WT)}")
        print(f"  - Background travel time variables (t_bg): {len(t_bg)}")

    # --- Phase 6: Build constraint blocks ---
    if verbose:
        print("[Phase 6] Building constraint blocks...")
    
    constraint_count = 0

    # Helper: paths by (o, d, m)
    paths_by_odm: Dict[Tuple[int, int, str], List[PathId]] = {}
    for pid in path_dict:
        o, d, m, _ = pid
        paths_by_odm.setdefault((o, d, m), []).append(pid)

    # Bus routes available for each OD (used to gate bus mode and flows)
    bus_routes_by_od: Dict[Tuple[int, int], List[int]] = {}
    for pid in path_dict:
        o, d, m, route_id = pid
        if m == "B":
            bus_routes_by_od.setdefault((o, d), []).append(route_id)

    def od_total(o: int, d: int) -> float:
        return sum(D_odh.get((o, d, h), 0.0) for h in H)

    def big_m_w(o: int, d: int) -> float:
        """Big-M for OD (o,d); prefer provided M_w, fallback to total demand."""
        M_w_param = params.get("M_w", {})
        return M_w_param.get((o, d), od_total(o, d))

    # --- Lower-level Primal Feasibility ---
    # Link flow definition (with PCE conversion for congestion modeling)
    # v_a = ∑_{w,m,k} f_k^{w,m} * δ_ak * (η_m / O_m) + background_flow
    # This converts passenger flows to vehicle PCE units for BPR travel time function
    for a in A:
        flow_sum = 0.0
        for pid in incidence.get(a, []):
            o, d, m, _ = pid
            # Get mode-specific PCE factor and occupancy from parameters
            eta_m = params.get("eta", {}).get(m, 1.0)  # Default: 1.0 PCE per pax
            O_m = params.get("O", {}).get(m, 1.0)      # Default: 1.0 pax per veh
            pce_factor = eta_m / O_m
            flow_sum = flow_sum + f[pid] * pce_factor
        
        background_flow = params.get("background_flow", {}).get(a, 0.0)
        mdl.add_constraint(
            v[a] == flow_sum + background_flow,
            ctname=f"link_flow_{a}"
        )
        constraint_count += 1
    
    # Mode-specific link flow definition: v_am[a][m] = sum of path flows for mode m on link a (in PCE units)
    for a in A:
        for m in M:
            paths_on_a_m = [pid for pid in incidence.get(a, []) if pid[2] == m]
            if paths_on_a_m:
                # Get mode-specific PCE factor and occupancy
                eta_m = params.get("eta", {}).get(m, 1.0)
                O_m = params.get("O", {}).get(m, 1.0)
                pce_factor = eta_m / O_m
                mdl.add_constraint(
                    v_am[a][m] == pce_factor * mdl.sum(f[pid] for pid in paths_on_a_m),
                    ctname=f"link_flow_mode_{a}_{m}"
                )
            else:
                # No paths of mode m use link a; force flow to zero
                mdl.add_constraint(
                    v_am[a][m] == 0.0,
                    ctname=f"link_flow_mode_{a}_{m}"
                )
            constraint_count += 1

    # ===== RULE 1 FIX: Path-to-mode conservation with Auto/Choice separation =====
    # For Auto (D): path flows equal fixed demand (no q variable)
    # For Choice modes: path flows equal q variable (elastic demand)
    for (o, d, m), plist in paths_by_odm.items():
        if m in M_Fixed:
            # Auto: direct demand allocation without q variable
            # Fixed auto demand = auto_demand_share × total OD demand
            total_od_demand = mdl.sum(D_odh.get((o, d, h), 0.0) for h in H)
            auto_fixed_demand = auto_demand_share * total_od_demand
            mdl.add_constraint(
                mdl.sum(f[pid] for pid in plist) == auto_fixed_demand,
                ctname=f"path_to_mode_fixed_{o}_{d}_{m}"
            )
        else:
            # Choice modes: path flows equal q (elastic demand via MNL)
            mdl.add_constraint(
                mdl.sum(f[pid] for pid in plist) == mdl.sum(q[(o, d, h, m)] for h in H if (o, d, h, m) in q_keys),
                ctname=f"path_to_mode_{o}_{d}_{m}"
            )
        constraint_count += 1

    # ===== RULE 1 FIX: Demand conservation for Choice modes only =====
    # Auto demand is handled separately (fixed)
    # Choice modes share the remaining demand pool: (1 - auto_share) × total
    for (o, d) in K:
        for h in H:
            total_od_h_demand = D_odh.get((o, d, h), 0.0)
            auto_od_h_demand = auto_demand_share * total_od_h_demand
            elastic_od_h_demand = (1.0 - auto_demand_share) * total_od_h_demand
            
            # Elastic demand constraint: Choice modes sum to elastic pool
            choice_modes_available = [m for m in M_h[h] if m in M_Choice]
            if choice_modes_available:
                mdl.add_constraint(
                    mdl.sum(q[(o, d, h, m)] for m in choice_modes_available if (o, d, h, m) in q_keys) == elastic_od_h_demand,
                    ctname=f"demand_elastic_{o}_{d}_{h}"
                )
                constraint_count += 1

    # Bus path activation: no bus flow if the corresponding route is inactive
    for pid in path_dict:
        o, d, m, route_id = pid
        if m == "B":
            mdl.add_constraint(
                f[pid] <= big_m_w(o, d) * x[route_id],
                ctname=f"bus_path_active_{route_id}_{o}_{d}"
            )
            constraint_count += 1

    # Bus mode availability: bus demand allowed only when at least one bus route for the OD is active
    for (o, d, h, m) in q_keys:
        if m == "B":
            routes = bus_routes_by_od.get((o, d), [])
            if routes:
                mdl.add_constraint(
                    q[(o, d, h, m)] <= big_m_w(o, d) * mdl.sum(x[r] for r in routes),
                    ctname=f"bus_mode_enable_{o}_{d}_{h}"
                )
                constraint_count += 1
            else:
                mdl.add_constraint(q[(o, d, h, m)] == 0, ctname=f"bus_mode_disabled_{o}_{d}_{h}")
                constraint_count += 1

    # Headway choice: exactly one headway when route is active (and none when inactive)
    if k_indices:
        for r in r_indices:
            mdl.add_constraint(
                mdl.sum(w[(r, k)] for k in k_indices) == x[r],
                ctname=f"headway_select_{r}"
            )
            constraint_count += 1

    # Bus capacity constraint (link-based): bus load on each edge cannot exceed offered capacity
    C_B = params.get("C_B", 40)  # Bus capacity (passengers per vehicle)
    
    # Helper: total bus flow for each route (sum of all bus path flows for that route)
    def get_route_flow(r: int) -> object:
        """Get total flow on route r (sum of flows on all bus paths of route r)."""
        route_flow = 0.0
        for pid in path_dict:
            o, d, m, route_id = pid
            if m == "B" and route_id == r:
                route_flow = route_flow + f[pid]
        return route_flow
    
    # New constraint: fleet size must be sufficient to handle the route demand
    # Constraint: n_r * C_B >= total bus flow on route r
    # This ensures capacity is not exceeded
    for r in r_indices:
        route_flow = get_route_flow(r)
        mdl.add_constraint(
            n[r] * C_B >= route_flow,
            ctname=f"bus_fleet_capacity_{r}"
        )
        constraint_count += 1
    
    # Fleet size only exists if route is active: n_r <= M * x_r
    max_fleet = sum(D_odh.values()) / C_B + 10  # Big-M upper bound on fleet
    for r in r_indices:
        mdl.add_constraint(
            n[r] <= max_fleet * x[r],
            ctname=f"bus_fleet_activation_{r}"
        )
        constraint_count += 1
    
    # Build incidence: which bus paths and routes traverse each edge
        bus_paths_by_edge: Dict[Tuple[int, int], List[PathId]] = {}
        bus_routes_by_edge: Dict[Tuple[int, int], set] = {}

        for pid, nodes in path_dict.items():
            o, d, m, route_id = pid
            if m != "B":
                continue
            for a in zip(nodes[:-1], nodes[1:]):
                bus_paths_by_edge.setdefault(a, []).append(pid)
                bus_routes_by_edge.setdefault(a, set()).add(route_id)

        # Old link-based constraint (now replaced by fleet capacity constraint above)
        # Kept as alternative: uncomment if needed
        # for a, bus_pids in bus_paths_by_edge.items():
        #     edge_flow = mdl.sum(f[pid] for pid in bus_pids)
        #     edge_capacity = mdl.sum(
        #         w[(r, k)] / headway_dict[k] * C_B if headway_dict[k] > 0 else 0
        #         for r in bus_routes_by_edge.get(a, set())
        #         for k in k_indices
        #     )
        #     mdl.add_constraint(
        #         edge_flow <= edge_capacity,
        #         ctname=f"bus_capacity_edge_{a[0]}_{a[1]}"
        #     )
        #     constraint_count += 1

    # --- Outer Approximation (Linearization) ---
    # Beckmann integral approximation (unified per-link with mode-weighted beta_time)
    # tau[a] ≥ (∑_m β_time^m) * [slope * (v_a - v_a^r) + intercept] for each breakpoint r
    # where v_a is the total link flow (sum across all modes with PCE conversion)
    
    for a in A:
        tangents = beck_tangents_by_edge[a]
        # Compute the weighted beta_time: sum of mode-specific time coefficients for road modes
        beta_sum = sum(BETA_TT.get(m, 0.0) for m in ['D', 'X', 'B', 'R'] if m in M)
        
        for r_idx, (slope, intercept) in enumerate(tangents):
            v_bp = beck_bps[r_idx] * G.edges[a].get("capacity", 1.0)
            # Beckmann constraint with unified link flow and mode-weighted coefficient
            # tau[a] ≥ (∑_m β_time^m) * [slope * (v_a - v_bp) + intercept]
            mdl.add_constraint(
                tau[a] >= beta_sum * (slope * (v[a] - v_bp) + intercept),
                ctname=f"beck_{a}_{r_idx}"
            )
            constraint_count += 1
    # Path entropy approximation
    for pid in path_dict:
        for j_idx, (slope, intercept) in enumerate(path_tangents):
            mdl.add_constraint(
                phi[pid] >= slope * f[pid] + intercept,
                ctname=f"phi_{pid}_{j_idx}"
            )
            constraint_count += 1

    # Mode entropy approximation
    for key in q_keys:
        for j_idx, (slope, intercept) in enumerate(mode_tangents):
            mdl.add_constraint(
                xi[key] >= slope * q[key] + intercept,
                ctname=f"xi_{key}_{j_idx}"
            )
            constraint_count += 1

    # --- Dual Feasibility Constraints ---
    # NOTE: SKIPPED if relax_to_lp=True (LP relaxation has no KKT dual constraints)
    if not relax_to_lp:
        # Dual for v_a (link flow stationarity)
        # From document: rho_a <= sum_r (-sum_m beta_time^m) * t_a(v_a^r) * alpha_ar[r]
        # The tangents already include the BPR slope, we need to weight by mode-specific beta_time
        for a in A:
            tangents = beck_tangents_by_edge[a]
            # Compute weighted beta_time for road-based modes
            beta_sum = sum(BETA_TT.get(m, 0.0) for m in ['D', 'X', 'B', 'R'] if m in M)
            
            # For each link, sum weighted tangent contributions from all breakpoints
            dual_sum_list = []
            for r in range(len(tangents)):
                slope = tangents[r][0]  # Already includes BPR slope: (-t0*alpha*beta*(ratio^(beta-1))/c)
                # Weight by sum of mode-specific beta_time values
                dual_sum_list.append(beta_sum * slope * alpha_ar[a][r])
            
            mdl.add_constraint(
                rho[a] - mdl.sum(dual_sum_list) <= 0,
                ctname=f"dual_v_{a}"
            )
            constraint_count += 1

        # Dual for f_k (path flow stationarity)
        # From document: Ψ_k + ∑_a δ_ak ρ_a + ∑_j (1 + ln f_hat_j) β_kj ≥ λ_w^m
        # Rearranged: λ_w^m - ∑_a δ_ak ρ_a - ∑_j (1 + ln f_hat_j) β_kj ≤ 0
        # Note: Ψ_k is NOT included here (it appears in primal objective with fixed utility term)
        # The tangent format is: (slope, intercept) where slope = 1 + ln(f_hat), intercept = -f_hat
        for pid, nodes in path_dict.items():
            o, d, m, _ = pid
            # Convert to int to match lambda_wm_keys type
            o_int, d_int = int(o), int(d)
            
            # Path links
            path_links = [(nodes[i], nodes[i+1]) for i in range(len(nodes)-1)]
            
            # ρ dual values for path links (congestion marginal cost)
            delta_sum = mdl.sum(
                rho[(u, v2)] for u, v2 in path_links
                if (u, v2) in rho
            )
            
            # Entropy linearization dual: ∑_j (1 + ln f_hat_j) β_kj
            # path_tangents[j] = (1 + ln(f_hat_j), -f_hat_j)
            # So path_tangents[j][0] is already (1 + ln f_hat_j)
            entropy_dual_sum = mdl.sum(
                path_tangents[j][0] * beta_kj[(pid, j)]
                for j in range(len(path_tangents))
            )
            
            mdl.add_constraint(
                lambda_wm[(o_int, d_int, m)]
                - delta_sum
                - entropy_dual_sum
                <= 0,
                ctname=f"dual_f_{pid}"
            )
            constraint_count += 1

        # ===== RULE 1 FIX: Dual for q - ONLY for Choice modes =====
        # Dual for q_{o,d,h,m} (mode demand stationarity)
        # From document: -λ_w^m + γ_w + ∑_j (1 + ln q_hat_j) η_wmj ≥ 0
        # Note: Fixed utility Ψ_w^m is NOT included here (it's in objective)
        # 
        # IMPORTANT: This constraint is ONLY for Choice modes (q_keys)
        # Auto (D) does NOT have a q variable, so NO mode entropy dual constraint
        # Auto paths still have lambda_wm (from path-to-fixed-demand constraint)
        # but no mode-level entropy constraint
        for (o, d, h, m) in q_keys:  # q_keys contains ONLY Choice modes
            # Convert to int to match lambda_wm_keys type
            o_int, d_int = int(o), int(d)
            
            # Check if this (o,d,m) combination exists in lambda_wm
            # (Some OD pairs might not have paths for certain modes)
            if (o_int, d_int, m) not in lambda_wm:
                continue  # Skip if no paths exist for this mode on this OD
            
            # Mode entropy linearization dual: ∑_j (1 + ln q_hat_j) η_wmj
            # mode_tangents[j] = (1 + ln(q_hat_j), -q_hat_j)
            # So mode_tangents[j][0] is already (1 + ln q_hat_j)
            entropy_dual_sum = mdl.sum(
                mode_tangents[j][0] * eta_wmj[(o, d, h, m, j)]
                for j in range(len(mode_tangents))
            )
            
            mdl.add_constraint(
                -lambda_wm[(o_int, d_int, m)]
                + gamma_w[(o, d)]
                + entropy_dual_sum
                >= 0,
                ctname=f"dual_q_{o}_{d}_{h}_{m}"
            )
            constraint_count += 1

        # Convexity constraints (sum of dual cut variables)
        for a in A:
            mdl.add_constraint(
                mdl.sum(alpha_ar[a][r] for r in range(len(beck_tangents_by_edge[a])))
                == 1.0,
                ctname=f"sum_alpha_{a}"
            )
            constraint_count += 1

        for pid in path_dict:
            mdl.add_constraint(
                mdl.sum(beta_kj[(pid, j)] for j in range(len(path_tangents)))
                == 1.0 / theta,
                ctname=f"sum_beta_{pid}"
            )
            constraint_count += 1

        # ===== RULE 1 FIX: Mode entropy constraint - ONLY for Choice modes =====
        # Auto (D) does NOT participate in MNL, so no entropy constraint for it
        for (o, d, h, m) in q_keys:  # q_keys now contains ONLY Choice modes
            mdl.add_constraint(
                mdl.sum(eta_wmj[(o, d, h, m, j)] for j in range(len(mode_tangents)))
                == 1.0 / mu,
                ctname=f"sum_eta_{o}_{d}_{h}_{m}"
            )
            constraint_count += 1

        # --- Strong Duality Equality ---
        # Compute weighted beta_time for Beckmann term
        beta_sum_for_beck = sum(BETA_TT.get(m, 0.0) for m in ['D', 'X', 'B', 'R'] if m in M)
        
        # ===== RULE 1 FIX: Primal objective - entropy ONLY for Choice modes =====
        # Primal objective: (∑_m β_time^m) * ∑_a tau[a] + entropy terms - fixed utility term
        # This represents the linearized lower-level objective:
        # ∑_a ∫_0^{v_a} (∑_m β_time^m) t_a(ξ) dξ + ∑_w,m,k f_k(-Ψ_k) + (1/θ)∑ phi + (1/μ)∑ xi
        # NOTE: xi now ONLY includes Choice modes (NOT Auto)
        primal_obj = (
            beta_sum_for_beck * mdl.sum(tau[a] for a in A)
            + (1.0 / theta) * mdl.sum(phi[pid] for pid in path_dict)
            + (1.0 / mu) * mdl.sum(xi[key] for key in q_keys)  # q_keys now only has Choice modes
            - mdl.sum(
                get_fixed_utility(params, o, d, h, m) * q[(o, d, h, m)]
                for (o, d, h, m) in q_keys  # Only Choice modes
            )
        )

        dual_obj = mdl.sum(od_total(o, d) * gamma_w[(o, d)] for (o, d) in K)

        # Add Beckmann dual contribution (NOT mode-specific, unified per-link)
        for a in A:
            tangents = beck_tangents_by_edge[a]
            beta_sum = sum(BETA_TT.get(m, 0.0) for m in ['D', 'X', 'B', 'R'] if m in M)
            for r_idx, (_, intercept) in enumerate(tangents):
                dual_obj += beta_sum * intercept * alpha_ar[a][r_idx]

        # Add entropy dual contributions
        for pid in path_dict:
            for j_idx, (_, intercept) in enumerate(path_tangents):
                dual_obj += intercept * beta_kj[(pid, j_idx)]

        # ===== RULE 1 FIX: Mode entropy dual - ONLY for Choice modes =====
        for (o, d, h, m) in q_keys:  # q_keys now only has Choice modes, so Auto is excluded
            for j_idx, (_, intercept) in enumerate(mode_tangents):
                dual_obj += intercept * eta_wmj[(o, d, h, m, j_idx)]

        mdl.add_constraint(
            primal_obj == dual_obj,
            ctname="strong_duality"
        )
        constraint_count += 1

    # --- McCormick Linearization Constraints (4-inequality envelope) ---
    if verbose:
        print("[Phase 6b] Adding McCormick linearization constraints...")
    
    # Helper function to get path free-flow time
    def get_path_time_hr(pid):
        """Get free-flow travel time for a path."""
        return compute_path_time_hr(G, path_dict[pid])
    
    # Compute bounds for McCormick envelope
    # Flow bounds: f_L = 0, f_U = total demand
    total_demand = sum(D_odh.values())
    f_L = 0.0
    f_U = total_demand
    
    # For auto modes: zeta_f_time[pid] linearizes f[pid] * t[pid]
    # McCormick envelope bounds for bilinear product: zeta = f * t
    # where f ∈ [f_L, f_U], t ∈ [t_L, t_U]
    # Standard form from literature:
    # (1) zeta >= f_L * t + t_L * f - f_L * t_L
    # (2) zeta >= f_U * t + t_U * f - f_U * t_U  
    # (3) zeta <= f_L * t + t_U * f - f_L * t_U
    # (4) zeta <= f_U * t + t_L * f - f_U * t_L
    #
    # For MILP with f_L=0, this simplifies to:
    # (1) zeta >= t_L * f
    # (2) zeta >= f_U * t + t_U * f - f_U * t_U
    # (3) zeta <= t_U * f  
    # (4) zeta <= f_U * t + t_L * f - f_U * t_L
    #
    # Since t is not an explicit variable (it's implicitly defined by link flows),
    # we use a conservative approximation by replacing t with its bounds:
    # - In constraint (2): use t ≥ t_L, giving: zeta >= f_U * t_L + t_U * f - f_U * t_U
    # - In constraint (4): use t ≤ t_U, giving: zeta <= f_U * t_U + t_L * f - f_U * t_L
    # This creates a valid outer approximation of the McCormick envelope.
    
    for pid in zeta_f_time.keys():
        o, d, m, _ = pid
        path_nodes = path_dict[pid]
        
        # Free-flow time (lower bound)
        t_L = get_path_time_hr(pid)
        
        # Upper bound: free-flow + estimated max congestion (e.g., 2x free-flow)
        t_U = t_L * 2.0
        
        # (1) zeta >= t_L * f (from constraint with f_L=0)
        mdl.add_constraint(
            zeta_f_time[pid] >= t_L * f[pid],
            ctname=f"mccormick_f_time_1_{pid}"
        )
        constraint_count += 1
        
        # (2) zeta >= f_U * t_L + t_U * f - f_U * t_U
        # Conservative: replaces t with t_L in the f_U * t term
        mdl.add_constraint(
            zeta_f_time[pid] >= f_U * t_L + t_U * f[pid] - f_U * t_U,
            ctname=f"mccormick_f_time_2_{pid}"
        )
        constraint_count += 1
        
        # (3) zeta <= t_U * f (from constraint with f_L=0)
        mdl.add_constraint(
            zeta_f_time[pid] <= t_U * f[pid],
            ctname=f"mccormick_f_time_3_{pid}"
        )
        constraint_count += 1
        
        # (4) zeta <= f_U * t_U + t_L * f - f_U * t_L
        # Conservative: replaces t with t_U in the f_U * t term
        mdl.add_constraint(
            zeta_f_time[pid] <= f_U * t_U + t_L * f[pid] - f_U * t_L,
            ctname=f"mccormick_f_time_4_{pid}"
        )
        constraint_count += 1
    
    # For bus in-vehicle time: zeta_f_TT[pid] linearizes f[pid] * TT[pid]
    for pid in zeta_f_TT.keys():
        o, d, m, route_id = pid
        path_nodes = path_dict[pid]
        
        # In-vehicle time bounds
        TT_L = get_path_time_hr(pid)  # Free-flow in-vehicle time
        TT_U = TT_L * 2.0  # With congestion
        
        # McCormick envelope constraints (4 inequalities)
        # (1) zeta >= TT_L * f
        mdl.add_constraint(
            zeta_f_TT[pid] >= TT_L * f[pid],
            ctname=f"mccormick_f_TT_1_{pid}"
        )
        constraint_count += 1
        
        # (2) zeta >= f_U * TT_L + TT_U * f - f_U * TT_U
        mdl.add_constraint(
            zeta_f_TT[pid] >= f_U * TT_L + TT_U * f[pid] - f_U * TT_U,
            ctname=f"mccormick_f_TT_2_{pid}"
        )
        constraint_count += 1
        
        # (3) zeta <= TT_U * f
        mdl.add_constraint(
            zeta_f_TT[pid] <= TT_U * f[pid],
            ctname=f"mccormick_f_TT_3_{pid}"
        )
        constraint_count += 1
        
        # (4) zeta <= f_U * TT_U + TT_L * f - f_U * TT_L
        mdl.add_constraint(
            zeta_f_TT[pid] <= f_U * TT_U + TT_L * f[pid] - f_U * TT_L,
            ctname=f"mccormick_f_TT_4_{pid}"
        )
        constraint_count += 1
    
    # For bus waiting time: zeta_f_WT[pid] linearizes f[pid] * WT[pid]
    # WT depends on headway selection (binary variable w)
    for pid in zeta_f_WT.keys():
        o, d, m, route_id = pid
        
        # Waiting time bounds
        WT_L = 0.0  # Minimum waiting time (highest frequency)
        if k_indices:
            WT_U = 0.5 * max(headway_dict.values())  # Maximum average waiting time
        else:
            WT_U = 0.5  # Default 30 minutes
        
        # Actual waiting time (depends on w)
        if k_indices and any((route_id, k) in w for k in k_indices):
            wt = 0.5 * mdl.sum(
                headway_dict[k] * w.get((route_id, k), 0)
                for k in k_indices
            )
        else:
            wt = 0.0
        
        # McCormick envelope for zeta = f * WT, f ∈ [0, f_U], WT ∈ [0, WT_U]
        # (1) zeta >= 0 * WT + 0 * f - 0 * 0 = 0
        mdl.add_constraint(
            zeta_f_WT[pid] >= 0,
            ctname=f"mccormick_f_WT_1_{pid}"
        )
        constraint_count += 1
        
        # (2) zeta >= f_U * WT + WT_U * f - f_U * WT_U
        # Since WT is expression with w, we need: zeta >= f_U * wt + WT_U * f - f_U * WT_U
        mdl.add_constraint(
            zeta_f_WT[pid] >= f_U * wt + WT_U * f[pid] - f_U * WT_U,
            ctname=f"mccormick_f_WT_2_{pid}"
        )
        constraint_count += 1
        
        # (3) zeta <= 0 * WT + WT_U * f - 0 * WT_U = WT_U * f
        mdl.add_constraint(
            zeta_f_WT[pid] <= WT_U * f[pid],
            ctname=f"mccormick_f_WT_3_{pid}"
        )
        constraint_count += 1
        
        # (4) zeta <= f_U * wt + 0 * f - f_U * 0
        # Since WT_L = 0, this simplifies to: zeta <= f_U * wt
        # But wt is itself a function of binary w, so this is valid
        mdl.add_constraint(
            zeta_f_WT[pid] <= f_U * wt,
            ctname=f"mccormick_f_WT_4_{pid}"
        )
        constraint_count += 1

    # Piecewise-linear BPR travel time for background flows
    bpr_params = params.get("bpr", {})
    bpr_alpha = bpr_params.get("alpha", BPR_ALPHA)
    bpr_beta = bpr_params.get("beta", BPR_BETA)

    for a in A:
        t0_hr = G.edges[a].get("free_flow_time", 0.0) / 3600.0
        capacity = G.edges[a].get("capacity", 1.0)

        for r_idx, ratio in enumerate(beck_bps):
            v_bp = ratio * capacity
            t_bp = t0_hr * (1.0 + bpr_alpha * (ratio ** bpr_beta))
            if capacity > 0:
                slope = t0_hr * bpr_alpha * bpr_beta * (ratio ** (bpr_beta - 1)) / capacity
            else:
                slope = 0.0

            # Supporting hyperplane: t_bg >= t(v_bp) + t'(v_bp)(v - v_bp)
            mdl.add_constraint(
                t_bg[a] >= t_bp + slope * (v[a] - v_bp),
                ctname=f"bpr_tt_bg_{a}_{r_idx}"
            )
            constraint_count += 1

    if verbose:
        print(f"  - Constraint blocks built: {constraint_count} constraints")

    # --- Phase 7: Upper-level objective (system cost) ---
    if verbose:
        print("[Phase 7] Building upper-level objective (system cost)...")
    
    FC_r = params.get("FC_r", 0.0)
    C_op_rk = params.get("C_op_rk_template", {})
    VOT_h = params.get("VOT_h", {})
    C_op_env_hat = params.get("C_op_env_hat", {"D": 0, "X": 0, "bg": 0})
    VOT_bg = params.get("VOT_bg", 0.0)
    
    # --- Component 1: System Operator Costs (Z_sys-op) ---
    # CRITICAL MODEL CHANGE: Operating cost is now based on FLEET SIZE (n_r), not FREQUENCY (w_rk)
    # 
    # Previous model: Z_op = FC_r*x_r + C_op_rk*w_rk (cost depended on frequency selection)
    # Current model:  Z_op = FC_r*x_r + C_fleet*n_r (cost depends on vehicle allocation)
    #
    # Rationale: In real transit operations:
    #   - Bus procurement/maintenance costs are tied to NUMBER OF VEHICLES (n_r), not schedule frequency
    #   - Frequency (w_rk) affects service quality, but vehicle cost is predetermined by fleet size
    #   - The constraints (capacity, activation, headway feasibility) ensure n_r is sufficient for chosen w_rk
    
    Z_sys_op_bus = mdl.sum(FC_r * x[r] for r in r_indices)
    
    # Fleet cost: cost per bus (purchase + maintenance) times number of buses
    # Get fleet cost parameters (default: 100 units per bus per year)
    # MODIFIED: Dramatically reduced fleet cost to encourage bus activation (was 100.0)
    fleet_cost_per_bus = params.get("fleet_cost_per_bus", 20.0)
    Z_sys_op_bus += mdl.sum(
        fleet_cost_per_bus * n[r]
        for r in r_indices
    )
    
    # NOTE: C_op_rk (frequency-dependent operating cost) is intentionally NOT included
    # The fleet size n_r now captures all vehicle-related operational expenses

    # Auto operational/environmental cost using McCormick variables
    # For driving (D) and taxi (X): sum over paths
    # Note: zeta already contains flow, so we don't multiply by demand again
    Z_sys_op_auto = 0.0
    for (o, d, m), plist in paths_by_odm.items():
        if m == "D":
            # Driving environmental cost
            for pid in plist:
                if pid in zeta_f_time:
                    Z_sys_op_auto += C_op_env_hat.get("D", 0.0) * zeta_f_time[pid]
        elif m == "X":
            # Taxi environmental cost
            for pid in plist:
                if pid in zeta_f_time:
                    Z_sys_op_auto += C_op_env_hat.get("X", 0.0) * zeta_f_time[pid]

    Z_sys_op = Z_sys_op_bus + Z_sys_op_auto

    # --- Component 2: User Costs (Z_user) ---
    # Using McCormick variables for travel time costs
    # Note: zeta_k = f_k * t_k already contains flow information
    # We need to weight by VOT for each segment
    Z_user = 0.0
    
    # Compute segment weights based on exogenous demand
    segment_weights = {}
    for (o, d) in K:
        total_demand = sum(D_odh.get((o, d, h), 0.0) for h in H)
        for h in H:
            if total_demand > 1e-9:
                segment_weights[(o, d, h)] = D_odh.get((o, d, h), 0.0) / total_demand
            else:
                segment_weights[(o, d, h)] = 1.0 / len(H)  # Equal weights if no demand
    
    # For each OD-mode pair, aggregate zeta with weighted VOT
    for (o, d, m), plist in paths_by_odm.items():
        # Compute weighted VOT for this OD pair
        weighted_vot = sum(
            segment_weights.get((o, d, h), 0.0) * VOT_h.get(h, 0.0)
            for h in H
        )
        
        if m in ["D", "X"]:
            # Auto modes: weighted_VOT * sum(zeta_f_time) + monetary cost per trip
            for pid in plist:
                if pid in zeta_f_time:
                    Z_user += weighted_vot * zeta_f_time[pid]
            
            # Add distance-based cost for D/X modes
            # Cost = f_k * cost_per_trip (where cost_per_trip depends on distance)
            if plist:
                o, d = plist[0][0], plist[0][1]  # Extract O-D from first path id
                cost_trip = params.get("Cost_od_auto", {}).get((o, d, m), 0.0)
                for pid in plist:
                    # Monetary user cost: per-trip $ cost × flow (no utility sign)
                    Z_user += f[pid] * cost_trip
        
        elif m == "B":
            # Bus mode: weighted_VOT * sum(zeta_f_TT + zeta_f_WT)
            for pid in plist:
                if pid in zeta_f_TT:
                    Z_user += weighted_vot * (zeta_f_TT[pid] + zeta_f_WT[pid])
            # Add unified bus fare per trip (monetary cost)
            fare_B = params.get("Fare_B", 0.0)
            if fare_B:
                for pid in plist:
                    Z_user += f[pid] * fare_B

    # Add costs for exogenous modes (R/W/O) based on fixed utility
    for (o, d, h, m) in q_keys:
        if m in ["R", "W", "O"]:
            # Use pre-computed fixed utility (V_bar) which includes time + cost
            psi_val = get_fixed_utility(params, o, d, h, m)
            vot = VOT_h.get(h, 0.0)
            # User cost = -VOT * Psi (Psi is negative utility, so -Psi is positive cost)
            Z_user += q[(o, d, h, m)] * (-vot * psi_val)

    # --- Component 3: Background Traffic Cost (Z_bg) ---
    # Using piecewise-linear BPR travel time (same breakpoints as Beckmann)
    bar_v_a = params.get("bar_v_a", {})
    Z_bg_terms = []
    for a in A:
        if bar_v_a.get(a, 0.0) > 0:
            Z_bg_terms.append(
                bar_v_a[a] * t_bg[a] * (VOT_bg + C_op_env_hat.get("bg", 0.0))
            )
    
    Z_bg = mdl.sum(Z_bg_terms) if Z_bg_terms else 0.0

    # Objective components
    obj_components = {
        "System_Operator_Cost": Z_sys_op,
        "User_Cost": Z_user,
        "Background_Traffic_Cost": Z_bg,
    }

    # Total objective
    mdl.minimize(mdl.sum(obj_components.values()))

    if verbose:
        print(f"  - Total variables: {mdl.number_of_variables}")
        print(f"  - Total constraints: {mdl.number_of_constraints}")

    return {
        "model": mdl,
        "variables": {
            "x": x,
            "w": w,
            "n": n,  # New: fleet size variables
            "v": v,
            "f": f,
            "q": q,
            "tau": tau,
            "phi": phi,
            "xi": xi,
            "rho": rho,
            "lambda": lambda_wm,
            "gamma": gamma_w,
            "alpha": alpha_ar,
            "beta": beta_kj,
            "eta": eta_wmj,
            "zeta_f_time": zeta_f_time,
            "zeta_f_TT": zeta_f_TT,
            "zeta_f_WT": zeta_f_WT,
            "t_bg": t_bg,
        },
        "obj_components": obj_components,
        "path_dict": path_dict,
        "P_hat": P_hat,
        "cycle_time_hr": cycle_time_hr,  # For reference
    }
