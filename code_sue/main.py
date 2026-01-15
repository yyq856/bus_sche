"""
Entry point for the SUE + MNL variant of the evacuation transit design model.

Data inputs remain identical to the existing pipeline; only the lower-level
behavioral equilibrium is changed to a stochastic user equilibrium with nested
MNL mode choice.
"""
from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Dict, List, Tuple

# Make the sibling "code" package importable (reuse data prep and parameters)
THIS_DIR = Path(__file__).resolve().parent
PROJECT_ROOT = THIS_DIR.parent
CODE_DIR = PROJECT_ROOT / "code"
if str(CODE_DIR) not in sys.path:
    sys.path.append(str(CODE_DIR))

from data_load import finalize_data_for_solver, load_model_data
from model_parameters import create_model_parameters

from .params import get_sue_hyperparams
from .sue_rmp import build_and_solve_sue_master


PathId = Tuple[int, int, str, int]  # (o, d, mode, path_index)


def _build_path_set(params: Dict) -> Dict[PathId, List[int]]:
    """
    Build a path dictionary keyed by (o, d, mode, path_idx).
    For now we reuse pre-generated auto paths for car/taxi; transit paths will be
    added by column generation later (stubs left in sue_rmp).
    """
    path_dict: Dict[PathId, List[int]] = {}
    auto_modes = ["D", "X"]
    for (o, d), paths in params.get("P_auto_od", {}).items():
        for m in auto_modes:
            for idx, node_list in enumerate(paths):
                path_dict[(o, d, m, idx)] = list(node_list)
    return path_dict


def run_sue_pipeline(
    data_pickle: str = "../data/model_input_data_10km.pkl",
    hyper_override: Dict | None = None,
    solve: bool = True,
):
    print("--- SUE + MNL pipeline (data unchanged) ---")
    data = load_model_data(data_pickle)
    if not data:
        print("Failed to load data. Abort.")
        return None

    solver_input = finalize_data_for_solver(data)
    if not solver_input:
        print("Data prep failed. Abort.")
        return None

    params = create_model_parameters(solver_input)
    path_dict = _build_path_set(params)
    hyper = get_sue_hyperparams(**(hyper_override or {}))

    print("Input summary: {} nodes, {} edges, {} OD pairs".format(
        len(params['N']), len(params['A']), len(params['K'])))
    print("Paths seeded for car/taxi:", len(path_dict))
    print("Hyper-parameters: theta={theta}, mu={mu}, big_M={big_m}".format(**hyper))

    result = build_and_solve_sue_master(params, path_dict, hyper, solve=solve)
    return result


if __name__ == "__main__":
    run_sue_pipeline(solve=True)
