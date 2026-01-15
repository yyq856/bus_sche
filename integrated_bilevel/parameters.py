"""
Parameters and hyperparameters for integrated bi-level model.

This module centralizes all configurable parameters:
- SUE/MNL dispersion parameters (theta, mu)
- Outer approximation breakpoints (Beckmann, entropy)
- Cost parameters (fixed costs, operating costs, VOT)
- Utility function parameters (beta coefficients)
"""

# ============================================================================
# SUE/MNL Dispersion Parameters
# ============================================================================

# Path choice dispersion (higher = more deterministic)
# Lower values increase randomness → more分散的路径选择
THETA = 0.3

# Mode choice dispersion (higher = more deterministic)
# Lower values increase randomness → 更分散的模式选择
MU = 0.3


# ============================================================================
# Outer Approximation Breakpoints
# ============================================================================

# Beckmann integral approximation (flow/capacity ratios)
# More breakpoints = better accuracy but larger model
BECKMANN_FLOW_BREAKPOINTS = [0.0, 0.5, 1.0, 1.5]

# Path flow entropy approximation (flow values)
# Format: [small_flow, medium_flow, large_flow]
ENTROPY_PATH_BREAKPOINTS = [1e-3, 0.1, 1.0]

# Mode demand entropy approximation (demand values)
ENTROPY_MODE_BREAKPOINTS = [1e-2, 0.1, 1.0]


# ============================================================================
# Cost Parameters - System Operator
# ============================================================================

# Fixed cost per route activation ($/route)
FIXED_COST_PER_ROUTE = 10.0

# Variable operating cost per vehicle-km ($/veh-km)
OPERATING_COST_PER_VEH_KM = 5.0

# Environmental cost by mode ($/pax-trip)
ENV_COST_HAT = {
    'D': 30.0,   # Drive alone
    'X': 35.0,   # Express/carpool
    'bg': 20.0   # Background traffic
}


# ============================================================================
# Cost Parameters - User Behavior
# ============================================================================

# Value of time by population segment ($/hr)
# Default values if not specified in data
VOT_DEFAULT = {
    'low': 15.0,
    'med': 20.0,
    'high': 25.0
}

# Background traffic value of time ($/hr)
VOT_BACKGROUND = 18.0

# ============================================================================
# Auto Mode Cost Parameters (D: Drive Alone, X: Taxi/Ride-sharing)
# ============================================================================

# Cost per trip for auto modes
# D (drive alone): $2.0/km (fuel + vehicle depreciation + maintenance + insurance)
# X (taxi): $3.0 base fare + $2.5/km
COST_COEFF_AUTO = {
    'D': -0.05,   # Drive alone: cost coefficient (disutility per dollar)
    'X': -0.10,   # Taxi: higher cost coefficient (more expensive)
}

# Fixed cost per trip (base fare, applies mainly to X)
AUTO_COST_BASE = {
    'D': 0.0,     # Drive alone: no base cost
    'X': 3.0,     # Taxi: $3 base fare
}

# Variable cost per km
AUTO_COST_PER_KM = {
    'D': 2.0,     # Drive alone: $2/km
    'X': 2.5,     # Taxi: $2.5/km
}


# ============================================================================
# Utility Function Parameters (MNL Mode Choice)
# ============================================================================

# Travel time coefficients by mode (utility per hour)
# Negative values indicate disutility (cost)
# Following standard literature (Fernández et al. 1994, de Cea & Fernández 1993)
# Values represent mode-specific marginal utility of travel time
BETA_TT = {
    'D': -3.0,   # Drive alone (highly sensitive to travel time)
    'X': -3.5,   # Express (taxi/rideshare, similar to driving)
    'B': -0.6,   # Bus (less sensitive to travel time than driving)
    'R': -0.5,   # Rail (least sensitive among motorized modes)
    'W': -2.0,   # Walk (moderately sensitive)
    'O': -1.6    # Other
}

# Mode-specific constants (base utility)
# MODIFIED: Greatly increased bus attractiveness and decreased other modes
BETA_CONST = {
    'D': 0.0,    # Drive alone (reference)
    'X': 0.2,
    'B': 0.8,    # MODIFIED: Bus (increased from 0.5 to 0.8 - very attractive)
    'R': -0.5,   # Rail
    'W': -3.5,   # MODIFIED: Walk (greatly reduced)
    'O': -1.0
}


# ============================================================================
# Network and Capacity Parameters
# ============================================================================

# BPR function parameters (Bureau of Public Roads)
BPR_ALPHA = 0.15  # Congestion sensitivity
BPR_BETA = 4.0    # Congestion steepness

# Minimum and maximum link capacities (veh/hr)
MIN_LINK_CAPACITY = 500.0
MAX_LINK_CAPACITY = 5000.0

# Default free-flow speed (km/hr) if not specified
DEFAULT_FREE_FLOW_SPEED = 50.0


# ============================================================================
# Bus Service Parameters
# ============================================================================

# Headway levels (minutes) - candidate frequencies
# Smaller values = more frequent service
HEADWAY_LEVELS = {
    1: 5.0,   # High frequency (5 min)
    2: 10.0,  # Medium-high (10 min)
    3: 15.0,  # Medium (15 min)
    4: 20.0,  # Medium-low (20 min)
    5: 30.0   # Low frequency (30 min)
}

# Bus capacity (passengers per vehicle)
BUS_CAPACITY = 40

# Average bus speed (km/hr) on dedicated lanes
BUS_SPEED_DEDICATED = 40.0

# Average bus speed (km/hr) in mixed traffic
BUS_SPEED_MIXED = 25.0


# ============================================================================
# Solver Configuration
# ============================================================================

# Default time limit (seconds)
DEFAULT_TIME_LIMIT = 3600

# Default MIP gap tolerance (relative)
DEFAULT_MIP_GAP = 0.05

# Number of threads (0 = use all available)
SOLVER_THREADS = 0

# Emphasis setting: 0=balanced, 1=feasibility, 2=optimality
SOLVER_EMPHASIS = 0


# ============================================================================
# Helper Functions
# ============================================================================

def get_sue_hyperparams():
    """
    Return SUE/MNL hyperparameters including breakpoints.
    
    Returns
    -------
    dict
        Contains keys: 'theta', 'mu', 'beckmann_flow_breakpoints', 
        'entropy_path_breakpoints', 'entropy_mode_breakpoints'
    """
    return {
        'theta': THETA,
        'mu': MU,
        'beckmann_flow_breakpoints': BECKMANN_FLOW_BREAKPOINTS,
        'entropy_path_breakpoints': ENTROPY_PATH_BREAKPOINTS,
        'entropy_mode_breakpoints': ENTROPY_MODE_BREAKPOINTS,
    }


def get_breakpoints():
    """
    Return all outer approximation breakpoints.
    
    Returns
    -------
    dict
        Contains keys: 'beckmann', 'path_entropy', 'mode_entropy'
    """
    return {
        'beckmann': BECKMANN_FLOW_BREAKPOINTS,
        'path_entropy': ENTROPY_PATH_BREAKPOINTS,
        'mode_entropy': ENTROPY_MODE_BREAKPOINTS
    }


def get_cost_params():
    """
    Return system cost parameters.
    
    Returns
    -------
    dict
        Contains keys: 'fixed_cost', 'operating_cost', 'env_cost', 'vot_default', 'vot_bg'
    """
    return {
        'fixed_cost': FIXED_COST_PER_ROUTE,
        'operating_cost': OPERATING_COST_PER_VEH_KM,
        'env_cost': ENV_COST_HAT,
        'vot_default': VOT_DEFAULT,
        'vot_bg': VOT_BACKGROUND
    }


def get_utility_params():
    """
    Return MNL utility function parameters.
    
    Returns
    -------
    dict
        Contains keys: 'beta_tt', 'beta_const'
    """
    return {
        'beta_tt': BETA_TT,
        'beta_const': BETA_CONST
    }


def get_all_params():
    """
    Return all parameters in a single dictionary.
    
    Returns
    -------
    dict
        Complete parameter set
    """
    return {
        'hyperparams': get_sue_hyperparams(),
        'breakpoints': get_breakpoints(),
        'cost': get_cost_params(),
        'utility': get_utility_params(),
        'bpr': {'alpha': BPR_ALPHA, 'beta': BPR_BETA},
        'bus': {
            'headway_levels': HEADWAY_LEVELS,
            'capacity': BUS_CAPACITY,
            'speed_dedicated': BUS_SPEED_DEDICATED,
            'speed_mixed': BUS_SPEED_MIXED
        },
        'solver': {
            'time_limit': DEFAULT_TIME_LIMIT,
            'mip_gap': DEFAULT_MIP_GAP,
            'threads': SOLVER_THREADS,
            'emphasis': SOLVER_EMPHASIS
        }
    }


# ============================================================================
# Tangent Generation Functions (from code_sue/params.py)
# ============================================================================

def build_beckmann_tangents(G, flow_breakpoints=None):
    """
    Build tangent lines for Beckmann integral approximation.
    
    The Beckmann function integrates link travel time:
        B(v) = ∫[0,v] t(x) dx
    where t(v) = t0 * (1 + alpha*(v/c)^beta) [BPR function]
    
    We approximate this via supporting hyperplanes (tangents) at breakpoints.
    
    Parameters
    ----------
    G : networkx.DiGraph
        Network graph with edge attributes 'free_flow_time', 'capacity'
    flow_breakpoints : list of float, optional
        Flow/capacity ratios for tangent breakpoints
        
    Returns
    -------
    dict
        edge -> list of (slope, intercept) tuples
    """
    if flow_breakpoints is None:
        flow_breakpoints = BECKMANN_FLOW_BREAKPOINTS
    
    tangents = {}
    
    for (u, v, data) in G.edges(data=True):
        t0 = data.get('free_flow_time', 1.0)
        capacity = data.get('capacity', 1000.0)
        
        edge_tangents = []
        for ratio in flow_breakpoints:
            v_bp = ratio * capacity
            
            # BPR travel time at breakpoint
            t_bp = t0 * (1.0 + BPR_ALPHA * (ratio ** BPR_BETA))
            
            # Derivative: dt/dv = t0 * alpha * beta * (v/c)^(beta-1) / c
            if ratio > 0:
                slope = t0 * BPR_ALPHA * BPR_BETA * (ratio ** (BPR_BETA - 1)) / capacity
            else:
                slope = 0.0
            
            # Beckmann value at breakpoint: B(v) = ∫[0,v] t(x) dx
            # For BPR: B(v) = t0*v + t0*alpha*c/(beta+1) * (v/c)^(beta+1)
            beckmann_bp = t0 * v_bp + (t0 * BPR_ALPHA * capacity / (BPR_BETA + 1)) * (ratio ** (BPR_BETA + 1))
            
            # Tangent line: B(v) ≥ slope*(v - v_bp) + beckmann_bp
            intercept = beckmann_bp - slope * v_bp
            
            edge_tangents.append((slope, intercept))
        
        tangents[(u, v)] = edge_tangents
    
    return tangents


def build_entropy_tangents(breakpoints):
    """
    Build tangent lines for entropy term: H(f) = f * ln(f).
    
    We use supporting hyperplanes (tangents) at breakpoints:
        f*ln(f) ≥ (1 + ln(f_bp)) * (f - f_bp) + f_bp*ln(f_bp)
                 = (1 + ln(f_bp)) * f - f_bp
    
    Parameters
    ----------
    breakpoints : list of float
        Flow/demand values for tangent breakpoints
        
    Returns
    -------
    list of tuple
        Each tuple: (slope, intercept) where slope = 1 + ln(bp), intercept = -bp
    """
    import math
    
    tangents = []
    for bp in breakpoints:
        if bp <= 0:
            # Skip non-positive breakpoints (entropy undefined)
            continue
        
        slope = 1.0 + math.log(bp)
        intercept = -bp
        
        tangents.append((slope, intercept))
    
    return tangents


if __name__ == "__main__":
    # Test parameter retrieval
    print("=== SUE Hyperparameters ===")
    print(get_sue_hyperparams())
    
    print("\n=== Breakpoints ===")
    print(get_breakpoints())
    
    print("\n=== Cost Parameters ===")
    print(get_cost_params())
    
    print("\n=== Utility Parameters ===")
    print(get_utility_params())
    
    print("\n=== All Parameters ===")
    import json
    print(json.dumps(get_all_params(), indent=2))
