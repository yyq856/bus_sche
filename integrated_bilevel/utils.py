#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Utility functions for integrated bi-level model.
Includes path processing, incidence matrices, tangent generation, etc.
"""
from typing import Dict, List, Tuple
import networkx as nx

PathId = Tuple[int, int, str, int]  # (origin, dest, mode, path_idx)


def compute_path_time_hr(G: nx.DiGraph, nodes: List[int]) -> float:
    """
    Compute free-flow travel time (in hours) for a sequence of nodes.
    
    Args:
        G: Network graph
        nodes: List of node IDs along the path
        
    Returns:
        Travel time in hours
    """
    total_sec = 0.0
    for u, v in zip(nodes[:-1], nodes[1:]):
        if (u, v) in G.edges:
            total_sec += G.edges[(u, v)].get("free_flow_time", 0.0)
    return total_sec / 3600.0


def build_initial_bus_paths(params: Dict) -> Dict[int, Dict]:
    """
    Generate initial bus paths using free-flow shortest paths per OD pair.
    
    Args:
        params: Model parameters dict with 'graph' and 'K' (OD pairs)
        
    Returns:
        Dict mapping path_id -> {nodes, route_id, od}
    """
    G = params["graph"]
    P_hat: Dict[int, Dict] = {}
    path_id = 0
    
    for o, d in params["K"]:
        try:
            nodes = nx.shortest_path(G, source=o, target=d, weight="free_flow_time")
            P_hat[path_id] = {
                "nodes": nodes,
                "route_id": path_id,
                "od": (o, d),
            }
            path_id += 1
        except (nx.NetworkXNoPath, nx.NodeNotFound):
            continue
    
    return P_hat


def build_path_dict(params: Dict, P_hat: Dict[int, Dict]) -> Dict[PathId, List[int]]:
    """
    Aggregate auto (D/X) and bus (B) paths into unified path_dict for SUE.
    
    Args:
        params: Model parameters with P_auto_od (auto paths) and M_h (modes)
        P_hat: Bus paths dict
        
    Returns:
        Unified path_dict: (o, d, mode, path_idx) -> node_list
    """
    path_dict: Dict[PathId, List[int]] = {}
    
    # Auto modes: use same paths for D and X (Drive, taxi-like)
    p_idx = 0
    for (o, d), paths in params.get("P_auto_od", {}).items():
        for k, nodes in enumerate(paths):
            # Both D and X use the same paths (simplification)
            path_dict[(o, d, "D", p_idx)] = nodes
            path_dict[(o, d, "X", p_idx)] = nodes
            p_idx += 1
    
    # Bus paths from P_hat
    for pid, pdata in P_hat.items():
        o, d = pdata["od"]
        path_dict[(o, d, "B", pid)] = pdata["nodes"]
    
    return path_dict


def build_link_path_incidence(path_dict: Dict[PathId, List[int]]) -> Dict[Tuple[int, int], List[PathId]]:
    """
    Build link-to-paths incidence map.
    
    Args:
        path_dict: Unified path dictionary
        
    Returns:
        Dict mapping edge (u,v) -> list of path IDs using that edge
    """
    incidence: Dict[Tuple[int, int], List[PathId]] = {}
    for pid, nodes in path_dict.items():
        for u, v in zip(nodes[:-1], nodes[1:]):
            incidence.setdefault((u, v), []).append(pid)
    return incidence


def get_fixed_utility(params: Dict, o: int, d: int, h: str, m: str) -> float:
    """
    Get fixed utility component Psi for mode m (from params or default).
    
    Args:
        params: Model parameters
        o, d: OD pair
        h: Population segment
        m: Mode
        
    Returns:
        Fixed utility value
    """
    if "V_bar_odhm" in params and (o, d, h, m) in params["V_bar_odhm"]:
        return float(params["V_bar_odhm"][(o, d, h, m)])
    return float(params.get("beta_const", {}).get(m, 0.0))


def aggregate_demand_per_mode(sol, all_vars: Dict, params: Dict, path_dict: Dict, P_hat: Dict) -> Dict[str, float]:
    """
    Aggregate demand across modes from solution.
    
    Args:
        sol: Solution object
        all_vars: Dict of variable dictionaries {q, y, z, ...}
        params: Model parameters
        path_dict: Unified path dictionary
        P_hat: Bus path dictionary
        
    Returns:
        Dict mapping mode -> total demand (passengers)
    """
    q = all_vars.get("q", {})
    y = all_vars.get("y", {})
    demand_per_mode = {}
    
    # Aggregate by mode from q (note: q is already absolute demand, not share)
    q_sol = sol.get_value_dict(q) if q else {}
    for (o, d, h, m), demand_val in q_sol.items():
        if demand_val > 1e-6:
            demand_per_mode[m] = demand_per_mode.get(m, 0.0) + demand_val
    
    # Aggregate bus mode (B) separately if using path-level variables (y would be shares)
    if y:
        y_sol = sol.get_value_dict(y)
        for (i, h), share in y_sol.items():
            if share > 1e-6 and i in P_hat:
                o, d = P_hat[i]["od"]
                mode_demand = params["D_odh"].get((o, d, h), 0.0) * share
                demand_per_mode["B"] = demand_per_mode.get("B", 0.0) + mode_demand
    
    return demand_per_mode


def print_solution_summary(sol, params: Dict, obj_components: Dict, all_vars: Dict, 
                          P_hat: Dict, path_dict: Dict) -> None:
    """
    Print a formatted solution summary.
    
    Args:
        sol: Solution object from solver
        params: Model parameters
        obj_components: Dict of objective function components
        all_vars: Dict of variable dictionaries
        P_hat: Bus paths
        path_dict: Unified paths
    """
    print("\n" + "="*70)
    print("INTEGRATED BI-LEVEL MODEL SOLUTION SUMMARY")
    print("="*70)
    
    if sol is None:
        print("No solution found.")
        return
    
    print(f"\nObjective Value: {sol.objective_value:,.2f}")
    print("\n--- Objective Breakdown ---")
    for name, expr in obj_components.items():
        try:
            val = sol.get_value(expr)
            print(f"  {name:<30}: {val:>15,.2f}")
        except Exception:
            pass
    
    print("\n--- Route Activation (x), Fleet Size (n), and Headway (w) ---")
    x = all_vars.get("x", {})
    w = all_vars.get("w", {})
    n = all_vars.get("n", {})
    if x:
        x_sol = sol.get_value_dict(x)
        activated = [r for r, v in x_sol.items() if v > 1e-3]
        if activated:
            print(f"  Activated routes: {len(activated)}")
            n_sol = sol.get_value_dict(n) if n else {}
            w_sol = sol.get_value_dict(w) if w else {}
            for r in activated[:10]:  # Show first 10
                print(f"    - Route {r}: x={x_sol[r]:.4f}", end="")
                # Show fleet size if available
                if n and r in n_sol:
                    n_buses = round(n_sol[r])
                    print(f", n_buses={n_buses}", end="")
                # Show selected headway if available
                if w:
                    selected_k = [k for (r_id, k), v in w_sol.items() if r_id == r and v > 0.5]
                    if selected_k:
                        print(f", Headway option: {selected_k[0]}")
                    else:
                        print()
                else:
                    print()
            if len(activated) > 10:
                print(f"    ... and {len(activated) - 10} more routes")
        else:
            print("  No routes activated.")
    
    print("\n--- Demand Distribution ---")    
    demand_per_mode = aggregate_demand_per_mode(sol, all_vars, params, path_dict, P_hat)
    total_demand = sum(params["D_odh"].values())
    for mode in sorted(demand_per_mode.keys()):
        demand = demand_per_mode[mode]
        pct = (demand / total_demand * 100) if total_demand > 0 else 0
        print(f"  {mode:<5}: {demand:>10,.0f} passengers ({pct:>6.2f}%)")
    
    print("="*70 + "\n")


def save_solution_to_file(sol, all_vars: Dict, P_hat: Dict, params: Dict,
                          obj_components: Dict, filename: str = "solution_result.json") -> None:
    """
    Save solution results to a JSON file.
    
    Args:
        sol: Solution object
        all_vars: Dict of variable dictionaries
        P_hat: Bus paths dictionary
        params: Model parameters
        obj_components: Objective components
        filename: Output filename
    """
    import json
    from pathlib import Path
    import numpy as np
    
    def convert_to_native(obj):
        """Convert numpy types to Python native types for JSON serialization."""
        if isinstance(obj, (np.integer, np.int64, np.int32)):
            return int(obj)
        elif isinstance(obj, (np.floating, np.float64, np.float32)):
            return float(obj)
        elif isinstance(obj, np.ndarray):
            return obj.tolist()
        elif isinstance(obj, dict):
            return {convert_to_native(k): convert_to_native(v) for k, v in obj.items()}
        elif isinstance(obj, (list, tuple)):
            return [convert_to_native(item) for item in obj]
        return obj
    
    if sol is None:
        print("No solution to save.")
        return
    
    result = {
        "objective_value": float(sol.objective_value),
        "objective_breakdown": {},
        "activated_routes": {},
        "headway_selection": {},
        "demand_distribution": {},
    }
    
    # Objective breakdown
    for name, expr in obj_components.items():
        try:
            val = sol.get_value(expr)
            result["objective_breakdown"][name] = float(val)
        except Exception:
            pass
    
    # Route activation and fleet size
    x = all_vars.get("x", {})
    n = all_vars.get("n", {})
    if x:
        x_sol = sol.get_value_dict(x)
        n_sol = sol.get_value_dict(n) if n else {}
        for r, v in x_sol.items():
            if v > 1e-3:
                route_info = P_hat.get(r, {})
                result["activated_routes"][str(r)] = {
                    "activation_value": float(v),
                    "fleet_size": int(round(n_sol.get(r, 0))) if n else None,
                    "od_pair": convert_to_native(route_info.get("od", None)),
                    "nodes": convert_to_native(route_info.get("nodes", [])),
                }
    
    # Headway selection
    w = all_vars.get("w", {})
    if w:
        w_sol = sol.get_value_dict(w)
        for (r, k), v in w_sol.items():
            if v > 0.5:
                if str(r) not in result["headway_selection"]:
                    result["headway_selection"][str(r)] = []
                result["headway_selection"][str(r)].append({
                    "headway_option": str(k),
                    "value": float(v)
                })
    
    # Demand distribution
    q = all_vars.get("q", {})
    if q:
        q_sol = sol.get_value_dict(q)
        total_demand = sum(params.get("D_odh", {}).values())
        mode_demand = {}
        # Note: q[(o,d,h,m)] is already absolute demand (passengers), not a share
        for (o, d, h, m), demand_val in q_sol.items():
            if demand_val > 1e-6:
                mode_demand[m] = mode_demand.get(m, 0.0) + demand_val
        
        for mode, demand in mode_demand.items():
            pct = (demand / total_demand * 100) if total_demand > 0 else 0
            result["demand_distribution"][mode] = {
                "passengers": float(demand),
                "percentage": float(pct)
            }
    
    # Save to file
    output_path = Path(filename)
    with open(output_path, "w", encoding="utf-8") as f:
        # Convert entire result dict to handle any nested numpy types
        json.dump(convert_to_native(result), f, indent=2, ensure_ascii=False)
    
    print(f"+ Solution saved to: {output_path.absolute()}")
