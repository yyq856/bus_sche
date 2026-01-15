#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Column Generation Framework for Bus Route Optimization.

Implements master-subproblem iteration:
  - Master Problem: Evaluates given set of bus routes
  - Subproblem: Generates new candidate bus routes
  - Iteration: Adds improving routes until no improvement
"""
from __future__ import annotations

import sys
import time
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Set
import networkx as nx

# Add code/ to path
code_path = str(Path(__file__).resolve().parent.parent / "code")
if code_path not in sys.path:
    sys.path.insert(0, code_path)

from data_load import load_model_data, finalize_data_for_solver  # type: ignore
from model_parameters import create_model_parameters  # type: ignore

try:
    from .parameters import get_sue_hyperparams
    from .model import build_integrated_bilevel_model
    from .utils import print_solution_summary
except ImportError:
    from parameters import get_sue_hyperparams
    from model import build_integrated_bilevel_model
    from utils import print_solution_summary


RouteId = Tuple[int, ...]  # Tuple of node IDs representing a route


class BusRoutePool:
    """
    Manages the pool of available bus routes.
    """
    
    def __init__(self, params: Dict):
        """
        Initialize with parameters.
        
        Args:
            params: Model parameters with graph and OD pairs
        """
        self.params = params
        self.graph = params['graph']
        self.od_pairs = params['K']
        
        # Route storage: route_id -> route data
        self.routes: Dict[int, Dict] = {}
        self.next_route_id = 0
        
        # Track which routes serve which OD pairs
        self.routes_by_od: Dict[Tuple[int, int], List[int]] = {
            od: [] for od in self.od_pairs
        }
        
    def add_route(self, nodes: List[int], od_pair: Tuple[int, int]) -> int:
        """
        Add a new bus route to the pool.
        
        Args:
            nodes: Sequence of node IDs defining the route
            od_pair: The (origin, destination) this route serves
            
        Returns:
            route_id: Assigned ID for this route
        """
        route_id = self.next_route_id
        self.next_route_id += 1
        
        self.routes[route_id] = {
            'nodes': nodes,
            'od_pair': od_pair,
            'route_id': route_id,
        }
        
        self.routes_by_od[od_pair].append(route_id)
        
        return route_id
    
    def get_route(self, route_id: int) -> Optional[Dict]:
        """Get route data by ID."""
        return self.routes.get(route_id)
    
    def get_all_routes(self) -> Dict[int, Dict]:
        """Get all routes."""
        return self.routes.copy()
    
    def get_routes_for_od(self, od_pair: Tuple[int, int]) -> List[int]:
        """Get all route IDs serving a specific OD pair."""
        return self.routes_by_od.get(od_pair, [])
    
    def num_routes(self) -> int:
        """Get total number of routes in pool."""
        return len(self.routes)
    
    def route_exists(self, nodes: List[int]) -> bool:
        """Check if a route with these nodes already exists."""
        nodes_tuple = tuple(nodes)
        for route_data in self.routes.values():
            if tuple(route_data['nodes']) == nodes_tuple:
                return True
        return False
    
    def remove_route(self, route_id: int) -> bool:
        """
        Remove a route from the pool.
        
        Args:
            route_id: ID of the route to remove
            
        Returns:
            True if route was removed, False if route_id not found
        """
        if route_id not in self.routes:
            return False
        
        # Get route info before removing
        route_data = self.routes[route_id]
        od_pair = route_data['od_pair']
        
        # Remove from main storage
        del self.routes[route_id]
        
        # Remove from OD index
        if od_pair in self.routes_by_od:
            self.routes_by_od[od_pair].remove(route_id)
        
        return True
    
    def clear_routes(self):
        """Remove all routes from the pool."""
        self.routes.clear()
        for od in self.routes_by_od:
            self.routes_by_od[od].clear()


def initialize_bus_routes(params: Dict) -> BusRoutePool:
    """
    Create initial set of bus routes (one shortest path per OD).
    
    Args:
        params: Model parameters
        
    Returns:
        BusRoutePool with initial routes
    """
    pool = BusRoutePool(params)
    G = params['graph']
    
    print("\n[Initializing Bus Routes]")
    for o, d in params['K']:
        try:
            # Find shortest path for this OD pair
            nodes = nx.shortest_path(G, source=o, target=d, weight='free_flow_time')
            route_id = pool.add_route(nodes, (o, d))
            print(f"  ✓ Route {route_id}: OD ({o}, {d}) with {len(nodes)} nodes")
        except nx.NetworkXNoPath:
            print(f"  ✗ No path found for OD ({o}, {d})")
    
    print(f"  Total initial routes: {pool.num_routes()}")
    return pool


def evaluate_with_routes(
    params: Dict,
    hyper: Dict,
    route_pool: BusRoutePool,
    time_limit: int = 3600,
    mip_gap: float = 0.05,
    verbose: bool = True,
) -> Optional[Dict]:
    """
    Evaluate objective function with given set of bus routes.
    
    This is the MASTER PROBLEM in column generation.
    
    Args:
        params: Model parameters
        hyper: Hyperparameters
        route_pool: Current pool of bus routes
        time_limit: Solver time limit
        mip_gap: MIP gap tolerance
        verbose: Print status
        
    Returns:
        Solution dict with objective value and variables
    """
    if verbose:
        print(f"\n[Evaluating with {route_pool.num_routes()} bus routes]")
    
    # Convert route pool to P_hat format expected by model builder
    # P_hat is now a dict: route_id -> route_data
    P_hat = route_pool.get_all_routes()
    
    # Update params with current routes
    params_copy = params.copy()
    params_copy['P_hat'] = P_hat
    
    try:
        # Build and solve model
        model_result = build_integrated_bilevel_model(
            params_copy, hyper, verbose=verbose
        )
        mdl = model_result["model"]
        
        # Set Gurobi parameters
        mdl.set_time_limit(time_limit)
        mdl.set_mip_gap(mip_gap)
        
        # Solve
        if verbose:
            print("\n[Solving]")
        solve_start = time.time()
        mdl.optimize()
        solve_time = time.time() - solve_start
        
        # Extract solution
        if mdl.is_optimal() or mdl.has_solution():
            obj_value = mdl.get_objective_value()
            
            if verbose:
                print(f"\n  ✓ Solution found!")
                print(f"  Objective value: {obj_value:.2f}")
                print(f"  Solve time: {solve_time:.2f}s")
            
            return {
                'objective': obj_value,
                'solve_time': solve_time,
                'status': 'optimal' if mdl.is_optimal() else 'feasible',
                'model': mdl,
                'variables': model_result['variables'],
                'route_pool': route_pool,
            }
        else:
            if verbose:
                print(f"\n  ✗ No solution found")
            return None
            
    except Exception as e:
        print(f"ERROR during evaluation: {e}")
        import traceback
        traceback.print_exc()
        return None


def generate_candidate_routes(
    params: Dict,
    route_pool: BusRoutePool,
    current_solution: Optional[Dict],
    k_routes: int = 3,
) -> List[Tuple[List[int], Tuple[int, int]]]:
    """
    Generate new candidate bus routes to add.
    
    This is the SUBPROBLEM in column generation.
    
    Strategy:
      - For each OD pair with <k routes, generate k-shortest paths
      - Filter out routes already in pool
      - Return top candidates
    
    Args:
        params: Model parameters
        route_pool: Current route pool
        current_solution: Current master problem solution (for dual info)
        k_routes: Number of routes to maintain per OD
        
    Returns:
        List of (nodes, od_pair) tuples for new candidate routes
    """
    G = params['graph']
    candidates = []
    
    print("\n[Generating Candidate Routes]")
    
    for o, d in params['K']:
        current_count = len(route_pool.get_routes_for_od((o, d)))
        
        if current_count >= k_routes:
            # Already have enough routes for this OD
            continue
        
        # Generate k-shortest paths
        try:
            paths_generator = nx.shortest_simple_paths(
                G, source=o, target=d, weight='free_flow_time'
            )
            
            added_for_od = 0
            for path in paths_generator:
                if added_for_od >= k_routes - current_count:
                    break
                
                # Check if route already exists
                if not route_pool.route_exists(path):
                    candidates.append((path, (o, d)))
                    added_for_od += 1
                    print(f"  + Candidate for OD ({o}, {d}): {len(path)} nodes")
                    
        except nx.NetworkXNoPath:
            print(f"  ✗ No additional paths for OD ({o}, {d})")
    
    print(f"  Total candidates generated: {len(candidates)}")
    return candidates


def column_generation_iteration(
    data_pickle_file: str = "../data/model_input_data_10km.pkl",
    max_iterations: int = 10,
    k_routes_per_od: int = 3,
    time_limit: int = 3600,
    mip_gap: float = 0.05,
    improvement_threshold: float = 0.01,
    verbose: bool = True,
) -> Optional[Dict]:
    """
    Main column generation loop.
    
    Process:
      1. Initialize with one shortest path per OD
      2. Solve master problem with current routes
      3. Generate new candidate routes (subproblem)
      4. Test each candidate: does it improve objective?
      5. Add improving routes to pool
      6. Repeat until no improvement or max iterations
    
    Args:
        data_pickle_file: Input data path
        max_iterations: Maximum CG iterations
        k_routes_per_od: Target number of routes per OD pair
        time_limit: Solver time limit per iteration
        mip_gap: MIP gap tolerance
        improvement_threshold: Minimum relative improvement to continue (%)
        verbose: Print status
        
    Returns:
        Final solution dict
    """
    start_time = time.time()
    
    print("="*80)
    print("COLUMN GENERATION FOR BUS ROUTE OPTIMIZATION")
    print("="*80)
    
    # --- Load Data ---
    print("\n[Loading Data]")
    try:
        data = load_model_data(data_pickle_file)
        solver_input = finalize_data_for_solver(data)
        params = create_model_parameters(solver_input)
        hyper = get_sue_hyperparams()
        
        print(f"  ✓ Network: {len(params['N'])} nodes, {len(params['A'])} edges")
        print(f"  ✓ OD pairs: {len(params['K'])}")
    except Exception as e:
        print(f"ERROR loading data: {e}")
        return None
    
    # --- Initialize Routes ---
    route_pool = initialize_bus_routes(params)
    
    # --- Column Generation Loop ---
    best_objective = float('inf')
    best_solution = None
    iteration = 0
    
    while iteration < max_iterations:
        iteration += 1
        print(f"\n{'='*80}")
        print(f"ITERATION {iteration}/{max_iterations}")
        print(f"{'='*80}")
        
        # Solve master problem with current routes
        solution = evaluate_with_routes(
            params, hyper, route_pool,
            time_limit=time_limit,
            mip_gap=mip_gap,
            verbose=verbose
        )
        
        if solution is None:
            print("  ✗ Master problem failed")
            break
        
        current_obj = solution['objective']
        
        # Check if improved
        if current_obj < best_objective:
            improvement = (best_objective - current_obj) / abs(best_objective) * 100
            best_objective = current_obj
            best_solution = solution
            print(f"\n  ✓ NEW BEST: {current_obj:.2f} (improvement: {improvement:.2f}%)")
        else:
            print(f"\n  No improvement: {current_obj:.2f} vs best {best_objective:.2f}")
        
        # Generate candidate routes
        candidates = generate_candidate_routes(
            params, route_pool, solution, k_routes=k_routes_per_od
        )
        
        if not candidates:
            print("\n  ✓ No more candidates - CONVERGED")
            break
        
        # Test each candidate
        print(f"\n[Testing {len(candidates)} Candidates]")
        added_count = 0
        
        for idx, (nodes, od_pair) in enumerate(candidates, 1):
            print(f"\n  Testing candidate {idx}/{len(candidates)} for OD {od_pair}...")
            
            # Add route temporarily
            route_id = route_pool.add_route(nodes, od_pair)
            
            # Evaluate
            test_solution = evaluate_with_routes(
                params, hyper, route_pool,
                time_limit=time_limit // 2,  # Less time for testing
                mip_gap=mip_gap,
                verbose=False
            )
            
            if test_solution and test_solution['objective'] < current_obj:
                improvement = (current_obj - test_solution['objective']) / abs(current_obj) * 100
                print(f"    ✓ IMPROVES: {test_solution['objective']:.2f} "
                      f"(+{improvement:.3f}%) - KEEPING route {route_id}")
                added_count += 1
                current_obj = test_solution['objective']
            else:
                # Remove route - didn't help
                print(f"    ✗ No improvement - REMOVING route {route_id}")
                del route_pool.routes[route_id]
                route_pool.routes_by_od[od_pair].remove(route_id)
        
        print(f"\n  Added {added_count} improving routes")
        print(f"  Total routes in pool: {route_pool.num_routes()}")
        
        if added_count == 0:
            print("\n  ✓ No improving routes found - CONVERGED")
            break
    
    # --- Final Results ---
    total_time = time.time() - start_time
    
    print("\n" + "="*80)
    print("COLUMN GENERATION COMPLETE")
    print("="*80)
    print(f"  Iterations: {iteration}")
    print(f"  Final routes: {route_pool.num_routes()}")
    print(f"  Best objective: {best_objective:.2f}")
    print(f"  Total time: {total_time:.2f}s")
    
    if best_solution:
        print("\n[Route Summary]")
        for route_id, route_data in route_pool.get_all_routes().items():
            od = route_data['od_pair']
            nodes = route_data['nodes']
            print(f"  Route {route_id}: OD {od} - {len(nodes)} nodes")
    
    return best_solution


if __name__ == "__main__":
    # Run column generation
    solution = column_generation_iteration(
        data_pickle_file="../data/model_input_data_10km.pkl",
        max_iterations=5,
        k_routes_per_od=3,
        time_limit=600,
        mip_gap=0.05,
        verbose=True,
    )
    
    if solution:
        print("\n✓ Optimization successful!")
    else:
        print("\n✗ Optimization failed!")
