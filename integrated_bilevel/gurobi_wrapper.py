#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Gurobi-based wrapper to convert docplex-style API calls to Gurobi.
This allows minimal changes to existing code structure.
"""
import gurobipy as gp
from gurobipy import GRB


class GurobiModelWrapper:
    """Wrapper to make Gurobi Model behave like docplex Model."""
    
    def __init__(self, name="model"):
        self.model = gp.Model(name)
        self._var_dict = {}
        
    def continuous_var_dict(self, keys, lb=0, ub=None, name="var"):
        """Create continuous variables similar to docplex."""
        if ub is None:
            ub = GRB.INFINITY
        if lb is None:
            lb = -GRB.INFINITY
            
        var_dict = {}
        if isinstance(keys, dict):
            keys = keys.keys()
        
        for key in keys:
            var_name = f"{name}_{key}" if isinstance(key, (int, str)) else f"{name}_{hash(key)}"
            var = self.model.addVar(lb=lb, ub=ub, name=var_name, vtype=GRB.CONTINUOUS)
            var_dict[key] = var
            
        self.model.update()
        return var_dict

    def binary_var_dict(self, keys, name="var"):
        """Create binary variables similar to docplex."""
        var_dict = {}
        if isinstance(keys, dict):
            keys = keys.keys()

        for key in keys:
            var_name = f"{name}_{key}" if isinstance(key, (int, str)) else f"{name}_{hash(key)}"
            var = self.model.addVar(lb=0, ub=1, name=var_name, vtype=GRB.BINARY)
            var_dict[key] = var

        self.model.update()
        return var_dict

    def integer_var_dict(self, keys, lb=0, ub=None, name="var"):
        """Create integer variables similar to docplex."""
        if ub is None:
            ub = GRB.INFINITY
        if lb is None:
            lb = 0
            
        var_dict = {}
        if isinstance(keys, dict):
            keys = keys.keys()

        for key in keys:
            var_name = f"{name}_{key}" if isinstance(key, (int, str)) else f"{name}_{hash(key)}"
            var = self.model.addVar(lb=lb, ub=ub, name=var_name, vtype=GRB.INTEGER)
            var_dict[key] = var

        self.model.update()
        return var_dict
    
    def sum(self, expr_list):
        """Sum expression compatible with Gurobi."""
        return gp.quicksum(expr_list)
    
    def add_constraint(self, constraint, ctname=""):
        """Add constraint to model."""
        self.model.addConstr(constraint, name=ctname)
        
    def minimize(self, expr):
        """Set minimization objective."""
        self.model.setObjective(expr, GRB.MINIMIZE)
        
    def set_time_limit(self, seconds):
        """Set time limit."""
        self.model.setParam('TimeLimit', seconds)
    
    def set_mip_gap(self, gap):
        """Set MIP gap tolerance."""
        self.model.setParam('MIPGap', gap)
    
    def optimize(self):
        """Optimize the model (Gurobi native)."""
        self.model.optimize()
    
    def is_optimal(self):
        """Check if solution is optimal."""
        return self.model.Status == GRB.OPTIMAL
    
    def has_solution(self):
        """Check if model has a solution."""
        return self.model.Status in [GRB.OPTIMAL, GRB.SUBOPTIMAL]
    
    def get_objective_value(self):
        """Get objective value."""
        try:
            return self.model.ObjVal
        except:
            return None
        
    def export_as_lp(self, filepath):
        """Export model as LP file."""
        try:
            self.model.write(str(filepath))
        except Exception as e:
            print(f"⚠ Could not export LP: {e}")
    
    def export(self, filepath):
        """Alias for export_as_lp."""
        self.export_as_lp(filepath)
            
    def solve(self, log_output=True):
        """Solve the model."""
        if not log_output:
            self.model.setParam('OutputFlag', 0)
        else:
            self.model.setParam('OutputFlag', 1)
            
        self.model.optimize()
        
        # Return a solution wrapper
        return GurobiSolutionWrapper(self.model)
    
    def update(self):
        """Update model."""
        self.model.update()
        
    @property
    def number_of_variables(self):
        """Get number of variables."""
        return self.model.NumVars
    
    @property
    def number_of_constraints(self):
        """Get number of constraints."""
        return self.model.NumConstrs


class GurobiSolutionWrapper:
    """Wrapper for Gurobi solution to match docplex interface."""
    
    def __init__(self, model):
        self.model = model
        self.status = model.Status
        
    @property
    def objective_value(self):
        """Get objective value."""
        try:
            return self.model.ObjVal
        except:
            return None
            
    def is_feasible(self):
        """Check if solution is feasible."""
        return self.status in [GRB.OPTIMAL, GRB.SUBOPTIMAL]
    
    def get_value_dict(self, var_dict):
        """Get solution values for a dictionary of variables."""
        result = {}
        for key, var in var_dict.items():
            try:
                result[key] = var.X
            except:
                result[key] = 0.0
        return result
    
    @property
    def solve_details(self):
        """Return solve details."""
        return GurobiSolveDetails(self.model)


class GurobiSolveDetails:
    """Wrapper for solve details."""
    
    def __init__(self, model):
        self.model = model
        
    @property
    def time(self):
        """Get solve time."""
        try:
            return self.model.Runtime
        except:
            return 0
            
    @property
    def gap(self):
        """Get MIP gap."""
        try:
            return self.model.MIPGap
        except:
            return 0


# Add solution_value property to Gurobi variables
def _get_solution_value(self):
    """Get solution value of variable."""
    try:
        return self.X
    except:
        return None

# Monkey patch Gurobi Var class
gp.Var.solution_value = property(_get_solution_value)
