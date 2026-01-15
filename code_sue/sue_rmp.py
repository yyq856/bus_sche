"""
SUE + MNL single-level reformulation via strong duality.

Implements the full bi-level to single-level transformation described in the
theoretical model, including outer approximation of convex terms, dual feasibility,
and strong duality equality with Big-M linearization.
"""
from __future__ import annotations

from typing import Dict, List, Tuple

from docplex.mp.model import Model

from .params import (
    build_beckmann_tangents,
    build_entropy_tangents,
)


PathId = Tuple[int, int, str, int]


def _build_link_path_incidence(path_dict: Dict[PathId, List[int]]) -> Dict[Tuple[int, int], List[PathId]]:
    """Map each directed edge to the set of path ids traversing it."""
    incidence: Dict[Tuple[int, int], List[PathId]] = {}
    for pid, nodes in path_dict.items():
        for u, v in zip(nodes[:-1], nodes[1:]):
            incidence.setdefault((u, v), []).append(pid)
    return incidence


def build_and_solve_sue_master(params: Dict, path_dict: Dict[PathId, List[int]], hyper: Dict, solve: bool = True):
    """
    Build the SUE+MNL single-level MILP via strong duality reformulation.
    
    Structure:
    1. Primal feasibility: link flow, path-mode conservation, demand conservation
    2. Outer approximations: Beckmann integral, path & mode entropy
    3. Dual feasibility: dual constraints for all primal variables
    4. Strong duality: equality constraint between primal and dual objectives
    """
    mdl = Model(name="SUE_MNL_RMP")

    # Problem data
    A = params["A"]  # edges
    K = params["K"]  # OD pairs
    H = params["H"]  # population segments
    M_h = params["M_h"]  # modes per segment
    D_odh = params["D_odh"]
    G = params["graph"]

    theta = float(hyper["theta"])
    mu = float(hyper["mu"])
    beck_breakpoints = hyper["beckmann_flow_breakpoints"]

    # Pre-build tangent lists (computed once)
    path_tangents = build_entropy_tangents(hyper["entropy_path_breakpoints"])
    mode_tangents = build_entropy_tangents(hyper["entropy_mode_breakpoints"])
    
    # Cache Beckmann tangents per edge
    beck_tangents_by_edge = {}
    for a in A:
        data = G.edges[a]
        capacity = data.get("capacity", 1.0)
        t0_hr = data.get("free_flow_time", 1.0) / 3600.0
        alpha = data.get("alpha", 0.15)
        beta_val = data.get("beta", 4.0)
        beck_tangents_by_edge[a] = build_beckmann_tangents(capacity, t0_hr, alpha, beta_val, beck_breakpoints)

    # --- PRIMAL VARIABLES ---
    y_edge = mdl.binary_var_dict(A, name="y_edge")
    v = mdl.continuous_var_dict(A, lb=0, name="v")
    f = mdl.continuous_var_dict(path_dict.keys(), lb=0, name="f")

    q_keys = [(o, d, h, m) for (o, d) in K for h in H for m in M_h[h]]
    q = mdl.continuous_var_dict(q_keys, lb=0, name="q")

    # Auxiliary variables for outer approximation
    tau = mdl.continuous_var_dict(A, lb=0, name="tau")
    phi = mdl.continuous_var_dict(path_dict.keys(), lb=0, name="phi")
    xi = mdl.continuous_var_dict(q_keys, lb=0, name="xi")

    incidence = _build_link_path_incidence(path_dict)

    # Helper mappings
    paths_by_odm: Dict[Tuple[int, int, str], List[PathId]] = {}
    for pid in path_dict:
        o, d, m, _ = pid
        paths_by_odm.setdefault((o, d, m), []).append(pid)

    def _od_total_demand(o: int, d: int) -> float:
        return sum(D_odh.get((o, d, h), 0.0) for h in H)

    def _psi(o: int, d: int, h: str, m: str) -> float:
        if "V_bar_odhm" in params and (o, d, h, m) in params["V_bar_odhm"]:
            return float(params["V_bar_odhm"][(o, d, h, m)])
        return float(params.get("beta_const", {}).get(m, 0.0))

    # --- PRIMAL FEASIBILITY CONSTRAINTS ---
    # 1. Link flow definition: v_a = sum of path flows using a
    for a in A:
        mdl.add_constraint(v[a] == mdl.sum(f[pid] for pid in incidence.get(a, [])), ctname=f"link_flow_{a}")

    # 2. Path-to-mode conservation: sum_paths for (o,d,m) = sum_h q_{o,d,h,m}
    for (o, d, m), pid_list in paths_by_odm.items():
        mdl.add_constraint(
            mdl.sum(f[pid] for pid in pid_list) == mdl.sum(q[(o, d, h, m)] for h in H),
            ctname=f"path_to_mode_{o}_{d}_{m}",
        )

    # 3. Demand conservation: sum_m q_{o,d,h,m} = D_{o,d,h}
    for (o, d) in K:
        for h in H:
            mdl.add_constraint(
                mdl.sum(q[(o, d, h, m)] for m in M_h[h]) == D_odh.get((o, d, h), 0.0),
                ctname=f"demand_{o}_{d}_{h}",
            )

    # --- OUTER APPROXIMATION CONSTRAINTS ---
    # Beckmann integral: tau_a >= t(v_bp) * (v_a - v_bp) + B(v_bp)
    for a in A:
        data = G.edges[a]
        capacity = data.get("capacity", 1.0)
        tangents = beck_tangents_by_edge[a]
        for r_idx, (slope, intercept) in enumerate(tangents):
            v_bp = beck_breakpoints[r_idx] * capacity
            mdl.add_constraint(
                tau[a] >= slope * (v[a] - v_bp) + intercept,
                ctname=f"beck_{a}_{r_idx}",
            )

    # Entropy approximation for paths: phi_pid >= (1 + ln bp) * f_pid - bp
    for pid in path_dict:
        for j_idx, (slope, intercept) in enumerate(path_tangents):
            mdl.add_constraint(
                phi[pid] >= slope * f[pid] + intercept,
                ctname=f"phi_{pid}_{j_idx}",
            )

    # Entropy approximation for modes: xi_{o,d,h,m} >= (1 + ln bp) * q - bp
    for key in q_keys:
        for j_idx, (slope, intercept) in enumerate(mode_tangents):
            mdl.add_constraint(
                xi[key] >= slope * q[key] + intercept,
                ctname=f"xi_{key}_{j_idx}",
            )

    # --- DUAL VARIABLES ---
    rho = mdl.continuous_var_dict(A, lb=None, ub=None, name="rho")
    
    # Lambda should cover all (o,d,m) combinations from q_keys, not just those with paths
    lambda_keys = list(set((o, d, m) for (o, d, h, m) in q_keys))
    lambda_wm = mdl.continuous_var_dict(lambda_keys, lb=None, ub=None, name="lambda")
    gamma_w = mdl.continuous_var_dict(K, lb=None, ub=None, name="gamma")

    alpha_ar = {}
    for a in A:
        tangents = beck_tangents_by_edge[a]
        alpha_ar[a] = mdl.continuous_var_dict(range(len(tangents)), lb=0, name=f"alpha_{a}")

    beta_kj = mdl.continuous_var_dict([(pid, j) for pid in path_dict for j in range(len(path_tangents))], lb=0, name="beta")
    eta_wmj = mdl.continuous_var_dict([(o, d, h, m, j) for (o, d, h, m) in q_keys for j in range(len(mode_tangents))], lb=0, name="eta")

    # --- DUAL FEASIBILITY CONSTRAINTS ---
    # For v_a: rho_a - sum_r slope * alpha_ar <= 0
    for a in A:
        tangents = beck_tangents_by_edge[a]
        mdl.add_constraint(
            rho[a] - mdl.sum(tangents[r][0] * alpha_ar[a][r] for r in range(len(tangents))) <= 0,
            ctname=f"dual_v_{a}",
        )

    # For f_pid: lambda_wm - sum_a delta * rho_a - sum_j slope * beta <= 0
    for pid, nodes in path_dict.items():
        o, d, m, _ = pid
        delta_sum = mdl.sum(rho[(u, v2)] for u, v2 in zip(nodes[:-1], nodes[1:]) if (u, v2) in rho)
        mdl.add_constraint(
            lambda_wm[(o, d, m)] - delta_sum - mdl.sum(path_tangents[j][0] * beta_kj[(pid, j)] for j in range(len(path_tangents))) <= 0,
            ctname=f"dual_f_{pid}",
        )

    # For q: -lambda_wm + gamma_w - sum_j slope * eta <= -psi
    for (o, d, h, m) in q_keys:
        psi_val = _psi(o, d, h, m)
        mdl.add_constraint(
            -lambda_wm[(o, d, m)] + gamma_w[(o, d)] - mdl.sum(mode_tangents[j][0] * eta_wmj[(o, d, h, m, j)] for j in range(len(mode_tangents))) <= -psi_val,
            ctname=f"dual_q_{o}_{d}_{h}_{m}",
        )

    # Sum constraints for dual cut variables
    for a in A:
        tangents = beck_tangents_by_edge[a]
        mdl.add_constraint(mdl.sum(alpha_ar[a][r] for r in range(len(tangents))) == 1.0, ctname=f"sum_alpha_{a}")

    for pid in path_dict:
        mdl.add_constraint(mdl.sum(beta_kj[(pid, j)] for j in range(len(path_tangents))) == 1.0 / theta, ctname=f"sum_beta_{pid}")

    for (o, d, h, m) in q_keys:
        mdl.add_constraint(mdl.sum(eta_wmj[(o, d, h, m, j)] for j in range(len(mode_tangents))) == 1.0 / mu, ctname=f"sum_eta_{o}_{d}_{h}_{m}")

    # --- STRONG DUALITY EQUALITY ---
    primal_obj = (
        mdl.sum(tau[a] for a in A)
        + (1.0 / theta) * mdl.sum(phi[pid] for pid in path_dict)
        + (1.0 / mu) * mdl.sum(xi[key] for key in q_keys)
        - mdl.sum(_psi(o, d, h, m) * q[(o, d, h, m)] for (o, d, h, m) in q_keys)
    )

    dual_obj = mdl.sum(_od_total_demand(o, d) * gamma_w[(o, d)] for (o, d) in K)
    
    # Add Beckmann dual contribution
    for a in A:
        tangents = beck_tangents_by_edge[a]
        for r_idx, (_, intercept) in enumerate(tangents):
            dual_obj += intercept * alpha_ar[a][r_idx]
    
    # Add entropy dual contributions
    for pid in path_dict:
        for j_idx, (_, intercept) in enumerate(path_tangents):
            dual_obj += intercept * beta_kj[(pid, j_idx)]
    
    for (o, d, h, m) in q_keys:
        for j_idx, (_, intercept) in enumerate(mode_tangents):
            dual_obj += intercept * eta_wmj[(o, d, h, m, j_idx)]

    mdl.add_constraint(primal_obj == dual_obj, ctname="strong_duality")

    # --- OBJECTIVE ---
    mdl.minimize(primal_obj)

    print("SUE+MNL model built: variables and constraints populated.")
    
    # --- SOLVE WITH GUROBI ---
    if solve:
        print("\nAttempting to solve with Gurobi...")
        print("  Note: Exporting LP file for native Gurobi solve...")
        
        try:
            # 先尝试Docplex原生求解器
            # 使用显式配置确保使用Gurobi
            solver_context = mdl.solve_with_parameters(
                name='gurobi',
                log_output=True,
                verbose=True,
            )
            
            # 如果上面失败，尝试导出LP文件给Gurobi原生求解
            if not mdl.solution:
                print("  Docplex solve failed, trying native Gurobi...")
                mdl.export_as_lp("sue_model_temp.lp")
                
                try:
                    import gurobipy as gp
                    env = gp.Env(empty=True)
                    env.setParam("OutputFlag", 1)
                    env.setParam("TimeLimit", 3600)
                    env.setParam("MIPGap", 0.05)
                    env.start()
                    
                    gurobi_mdl = gp.read("sue_model_temp.lp", env=env)
                    gurobi_mdl.optimize()
                    
                    if gurobi_mdl.status == 2:  # OPTIMAL
                        print("\n" + "="*60)
                        print("SOLUTION FOUND (via native Gurobi)")
                        print("="*60)
                        print(f"Objective value: {gurobi_mdl.objVal:.2f}")
                        
                        # Save solution file
                        gurobi_mdl.write("sue_gurobi_solution.sol")
                        print("Solution saved to sue_gurobi_solution.sol")
                except Exception as gurobi_e:
                    print(f"Native Gurobi also failed: {gurobi_e}")
            else:
                solution = mdl.solution
                print("\n" + "="*60)
                print("SOLUTION FOUND")
                print("="*60)
                print(f"Objective value: {mdl.objective_value:.2f}")
                
                # Print some key variable values
                print("\nKey variable values (sample):")
                v_sol = {a: solution.get_value(v[a]) for a in list(params["A"])[:5]}
                print(f"  Link flows (sample): {v_sol}")
                
                q_sample = {k: solution.get_value(q[k]) for k in list(q_keys)[:5]}
                print(f"  Mode shares (sample): {q_sample}")
                
                return {
                    "model": mdl,
                    "solution": solution,
                    "variables": {
                        "y_edge": y_edge,
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
                    },
                    "incidence": incidence,
                }
        except Exception as e:
            print(f"Solver error: {e}")
            import traceback
            traceback.print_exc()
            print("\nTip: Run solve_gurobi_native.py for native Gurobi solving")
    
    return {
        "model": mdl,
        "variables": {
            "y_edge": y_edge,
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
        },
        "incidence": incidence,
    }


__all__ = ["build_and_solve_sue_master"]
