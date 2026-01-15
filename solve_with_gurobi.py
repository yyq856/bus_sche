#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Solve the pre-built integrated bilevel model using Gurobi directly
"""

import gurobipy as gp
from gurobipy import GRB

def solve_lp_file(filename="integrated_bilevel.lp"):
    """Solve an existing LP file using Gurobi"""
    
    print("="*80)
    print("SOLVING INTEGRATED BILEVEL MODEL VIA GUROBI")
    print("="*80)
    
    print(f"\n[Step 1] Loading model from {filename}...")
    
    try:
        env = gp.Env()
        mdl = gp.read(filename, env=env)
        print("✓ Model loaded successfully")
        
        print(f"\nModel Information:")
        print(f"  Variables:     {mdl.NumVars}")
        print(f"  Constraints:   {mdl.NumConstrs}")
        print(f"  Non-zeros:     {mdl.NumNZs}")
        
        print(f"\n[Step 2] Setting optimization parameters...")
        mdl.setParam(GRB.Param.TimeLimit, 600)  # 10 minutes
        mdl.setParam(GRB.Param.OutputFlag, 1)   # Show progress
        mdl.setParam(GRB.Param.LogToConsole, 1)
        
        print(f"\n[Step 3] Optimizing model...")
        mdl.optimize()
        
        print(f"\n[Step 4] Retrieving solution...")
        
        if mdl.status == GRB.OPTIMAL:
            print(f"\n✓ OPTIMAL solution found!")
            print(f"  Objective Value: ${mdl.objVal:,.2f}")
        elif mdl.status == GRB.SUBOPTIMAL:
            print(f"\n⚠ FEASIBLE solution found (suboptimal)")
            print(f"  Objective Value: ${mdl.objVal:,.2f}")
        else:
            print(f"\n✗ Model status: {mdl.status}")
            return None
        
        print(f"\n" + "="*80)
        print("SOLUTION SUMMARY")
        print("="*80)
        print(f"Objective:      ${mdl.objVal:,.2f}")
        print(f"Status:         {['UNKNOWN', 'LOADED', 'OPTIMAL', 'INFEASIBLE', 'INF_OR_UNBD', 'UNBOUNDED'][mdl.status]}")
        print(f"Variables:      {mdl.NumVars}")
        print(f"Constraints:    {mdl.NumConstrs}")
        print(f"Solution Time:  {mdl.Runtime:.2f} seconds")
        print("="*80)
        
        return {
            'objective': mdl.objVal,
            'status': mdl.status,
            'is_optimal': (mdl.status == GRB.OPTIMAL),
            'runtime': mdl.Runtime,
        }
    
    except Exception as e:
        print(f"✗ Error: {e}")
        import traceback
        traceback.print_exc()
        return None

if __name__ == "__main__":
    result = solve_lp_file("integrated_bilevel.lp")
    
    if result:
        print(f"\n✓ Solution obtained successfully")
    else:
        print(f"\n✗ Failed to obtain solution")
