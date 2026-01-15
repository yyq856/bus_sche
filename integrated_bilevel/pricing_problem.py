#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Pricing Problem (Column Generation Subproblem) for Bus Route Generation.

Given dual prices from the master problem, generate new promising bus routes
with negative reduced cost.

Approach:
    1. Resource-Constrained Shortest Path (RCSP) with modified arc costs
    2. Arc cost = operational cost - dual revenue from serving demand
    3. Use dynamic programming or label-correcting algorithm
"""
from __future__ import annotations

import sys
from pathlib import Path
from typing import Dict, List, Tuple, Set, Optional
import networkx as nx
import numpy as np
from collections import defaultdict

# Add parent path for code/ imports
code_path = str(Path(__file__).resolve().parent.parent / "code")
if code_path not in sys.path:
    sys.path.insert(0, code_path)


class PricingProblem:
    """
    Subproblem for generating new bus routes with negative reduced cost.
    
    The reduced cost of a route r is:
        RC_r = c_r - Σ_{(o,d)} π_{od} · coverage_{r,od}
    
    where:
        c_r: operational cost of route r
        π_{od}: dual price for OD pair (o,d) from master problem
        coverage_{r,od}: binary indicator (1 if route r can serve OD pair (o,d))
    
    Goal: Find routes with RC_r < 0 (attractive to add to master problem)
    """
    
    def __init__(self, params: Dict, verbose: bool = False):
        """
        Initialize pricing problem.
        
        Args:
            params: Model parameters (graph, OD pairs, costs, etc.)
            verbose: Print debug information
        """
        self.G = params["graph"]
        self.K = params["K"]  # OD pairs
        self.verbose = verbose
        
        # Cost parameters
        self.base_route_cost = params.get("FC_r", 1000.0)  # Fixed cost per route
        self.fleet_cost_per_bus = params.get("fleet_cost_per_bus", 20.0)
        self.arc_cost_per_km = params.get("arc_cost_per_km", 0.5)
        
        # Route generation constraints
        self.max_route_length_km = params.get("max_route_length_km", 30.0)
        self.max_detour_ratio = params.get("max_detour_ratio", 1.5)
        self.min_demand_coverage = params.get("min_demand_coverage", 10.0)
        
        # Build auxiliary structures
        self._build_od_coverage_map()
        
    def _build_od_coverage_map(self):
        """
        Pre-compute which nodes can serve which OD pairs.
        A route serves an OD pair (o,d) if it passes through both o and d.
        """
        self.od_service_potential = defaultdict(set)
        
        for o, d in self.K:
            # Any route passing through both o and d can serve this OD
            self.od_service_potential[(o, d)] = {o, d}
    
    def compute_modified_arc_costs(self, dual_prices: Dict[Tuple[int, int], float]) -> Dict:
        """
        Compute modified arc costs incorporating dual prices.
        
        Arc cost = base_cost - potential_dual_revenue
        
        Args:
            dual_prices: Dict mapping (o,d) -> dual price π_{od}
            
        Returns:
            Dict mapping (u,v) -> modified_cost
        """
        modified_costs = {}
        
        for u, v in self.G.edges():
            # Base operational cost (proportional to distance)
            edge_data = self.G.edges[u, v]
            distance_km = edge_data.get("length", 1000.0) / 1000.0
            base_cost = self.arc_cost_per_km * distance_km
            
            # Potential dual revenue: sum of dual prices for OD pairs
            # that could be served by including this arc
            dual_revenue = 0.0
            for (o, d), price in dual_prices.items():
                # If this arc helps connect o to d, it captures dual revenue
                # Simple heuristic: arc (u,v) is valuable if u or v is close to o or d
                if u == o or v == d:
                    dual_revenue += price * 0.5  # Partial credit
                elif u == d or v == o:
                    dual_revenue += price * 0.3  # Reverse direction penalty
            
            modified_costs[(u, v)] = base_cost - dual_revenue
        
        return modified_costs
    
    def generate_k_shortest_paths(
        self,
        origin: int,
        destination: int,
        modified_costs: Dict,
        k: int = 5
    ) -> List[Tuple[List[int], float]]:
        """
        Generate K shortest paths from origin to destination using modified costs.
        
        Args:
            origin: Start node
            destination: End node
            modified_costs: Dict of modified arc costs
            k: Number of paths to generate
            
        Returns:
            List of (path_nodes, path_cost) tuples
        """
        try:
            # Use Yen's K shortest paths algorithm
            from networkx import shortest_simple_paths
            
            # Create a weighted graph with modified costs
            G_weighted = self.G.copy()
            for (u, v), cost in modified_costs.items():
                if (u, v) in G_weighted.edges:
                    G_weighted.edges[u, v]['modified_weight'] = max(cost, 0.01)
            
            paths = []
            for i, path in enumerate(shortest_simple_paths(
                G_weighted, origin, destination, weight='modified_weight'
            )):
                if i >= k:
                    break
                
                # Calculate path cost
                path_cost = sum(
                    modified_costs.get((path[j], path[j+1]), float('inf'))
                    for j in range(len(path) - 1)
                )
                
                paths.append((path, path_cost))
            
            return paths
        
        except Exception as e:
            if self.verbose:
                print(f"Warning: Could not generate paths from {origin} to {destination}: {e}")
            return []
    
    def solve(
        self,
        dual_prices: Dict[Tuple[int, int], float],
        max_routes: int = 10
    ) -> List[Dict]:
        """
        Solve pricing problem to generate new promising routes.
        
        Strategy:
            1. For each OD pair, generate K shortest paths with modified costs
            2. Evaluate reduced cost for each candidate route
            3. Return routes with negative reduced cost (most attractive first)
        
        Args:
            dual_prices: Dual prices from master problem {(o,d): price}
            max_routes: Maximum number of new routes to generate
            
        Returns:
            List of route dicts with keys:
                - 'nodes': List of nodes
                - 'reduced_cost': Reduced cost value
                - 'served_ods': Set of (o,d) pairs served
                - 'operational_cost': Route operational cost
        """
        if self.verbose:
            print(f"\n[Pricing Problem] Generating new routes...")
            print(f"  Dual prices provided: {len(dual_prices)}")
        
        # Step 1: Compute modified arc costs
        modified_costs = self.compute_modified_arc_costs(dual_prices)
        
        # Step 2: Generate candidate routes
        candidate_routes = []
        
        for o, d in self.K:
            # Generate K shortest paths for this OD pair
            paths = self.generate_k_shortest_paths(o, d, modified_costs, k=3)
            
            for path_nodes, path_cost in paths:
                # Evaluate this route
                route = self._evaluate_route(path_nodes, dual_prices)
                
                if route is not None:
                    candidate_routes.append(route)
        
        # Step 3: Sort by reduced cost (most negative first)
        candidate_routes.sort(key=lambda r: r['reduced_cost'])
        
        # Step 4: Return top routes with negative reduced cost
        attractive_routes = [
            r for r in candidate_routes[:max_routes]
            if r['reduced_cost'] < -0.01  # Small tolerance for numerical errors
        ]
        
        if self.verbose:
            print(f"  Generated {len(candidate_routes)} candidate routes")
            print(f"  Found {len(attractive_routes)} routes with negative reduced cost")
            if attractive_routes:
                best_rc = attractive_routes[0]['reduced_cost']
                print(f"  Best reduced cost: {best_rc:.4f}")
        
        return attractive_routes
    
    def _evaluate_route(
        self,
        nodes: List[int],
        dual_prices: Dict[Tuple[int, int], float]
    ) -> Optional[Dict]:
        """
        Evaluate a candidate route and compute its reduced cost.
        
        Args:
            nodes: Route node sequence
            dual_prices: Dual prices from master problem
            
        Returns:
            Route dict or None if route is invalid
        """
        # Check route validity
        if len(nodes) < 2:
            return None
        
        # Compute route length
        total_length_km = 0.0
        for i in range(len(nodes) - 1):
            u, v = nodes[i], nodes[i+1]
            if (u, v) not in self.G.edges:
                return None  # Invalid route
            edge_data = self.G.edges[u, v]
            total_length_km += edge_data.get("length", 1000.0) / 1000.0
        
        # Check length constraint
        if total_length_km > self.max_route_length_km:
            return None
        
        # Compute operational cost
        operational_cost = (
            self.base_route_cost +
            self.arc_cost_per_km * total_length_km
        )
        
        # Determine which OD pairs this route serves
        node_set = set(nodes)
        served_ods = set()
        dual_revenue = 0.0
        
        for (o, d), price in dual_prices.items():
            # Route serves (o,d) if both o and d are on the route
            # and d comes after o in the sequence
            if o in node_set and d in node_set:
                o_idx = nodes.index(o)
                try:
                    d_idx = nodes.index(d)
                    if d_idx > o_idx:  # Correct direction
                        served_ods.add((o, d))
                        dual_revenue += price
                except ValueError:
                    pass
        
        # Compute reduced cost
        reduced_cost = operational_cost - dual_revenue
        
        # Return route info
        return {
            'nodes': nodes,
            'reduced_cost': reduced_cost,
            'served_ods': served_ods,
            'operational_cost': operational_cost,
            'dual_revenue': dual_revenue,
            'length_km': total_length_km,
        }
    
    def solve_advanced(
        self,
        dual_prices: Dict[Tuple[int, int], float],
        lambda_bus_dual: Dict,  # Bus flow dual variables
        max_routes: int = 10
    ) -> List[Dict]:
        """
        Advanced pricing using both demand duals and flow duals.
        
        This version considers:
            1. Dual prices for OD coverage (γ_w)
            2. Path flow duals (λ_w^B) for bus mode
            3. Link congestion duals (ρ_a)
        
        Args:
            dual_prices: OD-level dual prices
            lambda_bus_dual: Bus mode dual variables from master problem
            max_routes: Maximum routes to return
            
        Returns:
            List of promising routes
        """
        # For now, use the basic solve method
        # Future enhancement: incorporate path and link duals
        return self.solve(dual_prices, max_routes)


def test_pricing_problem():
    """Test the pricing problem with synthetic data."""
    import networkx as nx
    
    # Create a simple test network
    G = nx.DiGraph()
    nodes = list(range(10))
    for u in nodes:
        for v in nodes:
            if u != v and abs(u - v) <= 2:
                G.add_edge(u, v, length=abs(u-v)*1000, free_flow_time=abs(u-v)*60)
    
    # Create test parameters
    params = {
        "graph": G,
        "K": [(0, 5), (1, 6), (2, 7)],
        "FC_r": 1000.0,
        "fleet_cost_per_bus": 20.0,
        "arc_cost_per_km": 0.5,
    }
    
    # Create pricing problem
    pp = PricingProblem(params, verbose=True)
    
    # Test with some dual prices
    dual_prices = {
        (0, 5): 500.0,
        (1, 6): 300.0,
        (2, 7): 200.0,
    }
    
    # Solve
    new_routes = pp.solve(dual_prices, max_routes=5)
    
    print(f"\nGenerated {len(new_routes)} new routes:")
    for i, route in enumerate(new_routes):
        print(f"\nRoute {i+1}:")
        print(f"  Nodes: {route['nodes']}")
        print(f"  Reduced cost: {route['reduced_cost']:.2f}")
        print(f"  Serves ODs: {route['served_ods']}")
        print(f"  Length: {route['length_km']:.2f} km")


if __name__ == "__main__":
    test_pricing_problem()
