# SUE + MNL Variant

This folder hosts a scaffolding for the stochastic user equilibrium (route) and
multinomial logit (mode) reformulation described in your theoretical note. Data
inputs stay identical to the original pipeline; only the lower-level equilibrium
changes.

## Files
- [code_sue/main.py](code_sue/main.py): entry point; loads existing data, builds the SUE hyper-params, and calls the new master builder.
- [code_sue/params.py](code_sue/params.py): centralized non-decision parameters (theta, mu, Big-M, Beckmann and entropy breakpoints) plus helper tangent builders.
- [code_sue/sue_rmp.py](code_sue/sue_rmp.py): Docplex scaffold that defines variables for the single-level SUE+MNL master; TODOs highlight where to add primal, dual, and strong-duality constraints.
- [code_sue/__init__.py](code_sue/__init__.py): convenience exports.

## How to start
1) Install Docplex if missing: `pip install docplex`
2) From the repo root, run the SUE pipeline (data file unchanged):
   - `python -m code_sue.main`
3) Fill the TODO blocks in [code_sue/sue_rmp.py](code_sue/sue_rmp.py) using your derivation:
   - Add primal feasibility: link flow definition, path-to-mode conservation, demand conservation.
   - Add Beckmann outer-approximation cuts via tangents from `params.build_beckmann_tangents`.
   - Add entropy outer-approximation cuts via tangents from `params.build_entropy_tangents`.
   - Add dual feasibility constraints and the strong-duality equality; linearize design*dual products with Big-M.

## Notes
- Paths: current scaffold reuses car and taxi shortest paths from the existing `P_auto_od` as seeds; extend with transit paths as columns are generated.
- Design variables: `y_edge` is a placeholder for edge-level design (e.g., bus lanes). Adjust or replace depending on your upper-level decisions.
- Hyper-parameters: edit once in [code_sue/params.py](code_sue/params.py) or override via `run_sue_pipeline(hyper_override=...)`.
