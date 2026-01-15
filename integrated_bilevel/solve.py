#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Integrated Bi-Level Problem Solver.

Loads data, builds model, solves with Gurobi, and reports results.
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Dict, Optional

# Add code/ to path for data pipeline
code_path = str(Path(__file__).resolve().parent.parent / "code")
if code_path not in sys.path:
    sys.path.insert(0, code_path)

from data_load import load_model_data, finalize_data_for_solver  # type: ignore
from model_parameters import create_model_parameters  # type: ignore

# Local imports from integrated_bilevel/
# Try relative import first, fall back to absolute
try:
    from .parameters import get_sue_hyperparams
    from .model import build_integrated_bilevel_model
    from .utils import print_solution_summary, save_solution_to_file
except ImportError:
    # If run as script, use absolute imports
    from parameters import get_sue_hyperparams
    from model import build_integrated_bilevel_model
    from utils import print_solution_summary, save_solution_to_file


def solve_bilevel_problem(
    data_pickle_file: str = "../data/model_input_data_10km.pkl",
    time_limit: int = 3600,
    mip_gap: float = 0.05,
    verbose: bool = True,
) -> Optional[Dict]:
    """
    Solve the integrated bi-level problem end-to-end.
    
    Args:
        data_pickle_file: Path to model data pickle file
        time_limit: Gurobi time limit (seconds)
        mip_gap: Gurobi MIP gap tolerance
        verbose: Print status messages
        
    Returns:
        Dict with solution info, or None if solve failed
    """
    start_time = time.time()

    # --- Data Loading ---
    if verbose:
        print("="*70)
        print("INTEGRATED BI-LEVEL PROBLEM SOLVER")
        print("="*70)
        print("\n[Loading Data]")
    
    try:
        data = load_model_data(data_pickle_file)
        if not data:
            print("ERROR: Failed to load data")
            return None
        
        solver_input = finalize_data_for_solver(data)
        if not solver_input:
            print("ERROR: Failed to finalize solver input")
            return None
        
        params = create_model_parameters(solver_input)
        if not params:
            print("ERROR: Failed to create parameters")
            return None
        
        hyper = get_sue_hyperparams()
        
        if verbose:
            print(f"  + Nodes: {len(params['N'])}")
            print(f"  + Edges: {len(params['A'])}")
            print(f"  + OD pairs: {len(params['K'])}")
            print(f"  + Population segments: {len(params['H'])}")
    
    except Exception as e:
        print(f"ERROR during data loading: {e}")
        import traceback
        traceback.print_exc()
        return None

    # --- Model Building ---
    if verbose:
        print("\n[Building Model]")
    
    try:
        model_result = build_integrated_bilevel_model(params, hyper, verbose=verbose)
        mdl = model_result["model"]
        all_vars = model_result["variables"]
        obj_components = model_result["obj_components"]
        path_dict = model_result["path_dict"]
        P_hat = model_result["P_hat"]
    
    except Exception as e:
        print(f"ERROR during model building: {e}")
        import traceback
        traceback.print_exc()
        return None

    # Export LP for inspection
    try:
        lp_file = Path("integrated_bilevel.lp")
        mdl.export(lp_file)
        if verbose:
            print(f"  + Exported to {lp_file}")
    except Exception as e:
        if verbose:
            print(f"  ⚠ Could not export LP: {e}")

    # --- Solving ---
    if verbose:
        print(f"\n[Solving with Gurobi]")
        print(f"  Time limit: {time_limit}s")
        print(f"  MIP gap: {mip_gap*100:.1f}%")
        print()
    
    solve_start = time.time()
    
    try:
        # Configure Gurobi parameters
        mdl.set_time_limit(time_limit)
        
        # Note: Docplex wraps Gurobi parameters
        # Use cplex.parameters if using CPLEX, else gurobi directly
        sol = mdl.solve(log_output=True)
        
        solve_time = time.time() - solve_start
        
        if verbose:
            print(f"\n[Solution Status]")
        
        if sol is None or not hasattr(sol, 'objective_value') or sol.objective_value is None:
            print("\n[ERROR] Solver returned no solution - Model is INFEASIBLE or UNBOUNDED")
            print("  Exporting IIS (Irreducible Inconsistent Subsystem) for debugging...")
            try:
                mdl.compute_iis()
                iis_file = Path("infeasible_model.ilp")
                mdl.write_iis(iis_file)
                print(f"  + IIS written to {iis_file}")
            except Exception as e:
                print(f"  Could not compute IIS: {e}")
            return None
        
        if verbose:
            print(f"  + Objective: {sol.objective_value:,.2f}")
            print(f"  + Solve time: {solve_time:.1f}s")
            print(f"  + Total time: {time.time() - start_time:.1f}s")
    
    except Exception as e:
        print(f"\n[ERROR] Solve failed: {e}")
        import traceback
        traceback.print_exc()
        return None

    # --- Results Summary ---
    if verbose:
        print_solution_summary(sol, params, obj_components, all_vars, P_hat, path_dict)
    
    # --- Save Results to File ---
    try:
        save_solution_to_file(sol, all_vars, P_hat, params, obj_components, 
                            filename="solution_result.json")
    except Exception as e:
        print(f"Warning: Could not save solution to file: {e}")

    return {
        "solution": sol,
        "model": mdl,
        "variables": all_vars,
        "obj_components": obj_components,
        "path_dict": path_dict,
        "P_hat": P_hat,
        "params": params,
        "solve_time": solve_time,
    }


def main():
    """Entry point for command-line execution."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Solve integrated bi-level SUE-MNL transportation network design"
    )
    parser.add_argument(
        "--data",
        type=str,
        default="../data/model_input_data_10km.pkl",
        help="Path to model data file"
    )
    parser.add_argument(
        "--time-limit",
        type=int,
        default=3600,
        help="Solver time limit (seconds)"
    )
    parser.add_argument(
        "--mip-gap",
        type=float,
        default=0.05,
        help="MIP gap tolerance"
    )
    parser.add_argument(
        "--verbose",
        action="store_true",
        default=True,
        help="Print detailed output"
    )
    
    args = parser.parse_args()
    
    result = solve_bilevel_problem(
        data_pickle_file=args.data,
        time_limit=args.time_limit,
        mip_gap=args.mip_gap,
        verbose=args.verbose,
    )
    
    if result is None:
        sys.exit(1)
    else:
        print("\n[SUCCESS] Problem solved!")
        sys.exit(0)


if __name__ == "__main__":
    main()
