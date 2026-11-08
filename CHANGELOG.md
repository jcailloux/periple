# Changelog

## Unreleased

### Added

- `time_limit` in the params of every algorithm: stopped early, it returns its best complete tour (`feasible`) or else a partial one (`partial`).
- Benchmark runner: `--time-limit MS`.
- `select_next(solver, stop)`: a construction strategy that runs long can honor `time_limit`.
- `branch_and_bound()`: exact depth-first search through the `AppendMove` pipeline, so it honors every variant and stays exact under time windows. A complete tour held by the solver is the first to beat, `resume_at` searches the completions of a prefix, `max_nodes` and `time_limit` stop it early.
- `branch_and_bound(bound)`: the same search with a lower bound strategy (`lower_bound`, optional `begin`). Built-in bounds: `SpanningTreeBound` for symmetric distances and `SpanningArborescenceBound` for any instance, Held-Karp Lagrangian relaxations ascended at every node and the defaults of `branch_and_bound()`; `AssignmentBound`, for costs without structure under time windows.
- `branch_and_bound(bound, order)`: the order of the children is a strategy too, through `order` (once per node, permutes the scored children) or `pick` (before each descent, the next child), or both. `CheapestFirst` is the default. `dead_end` is what a lower bound returns for a node without a feasible completion.

### Breaking

- Registry descriptors derive from `Algorithm<Derived>` and define `run(solver, RunOptions)`, plus `setup(solver)` for an improvement algorithm; `operator()` takes `RunOptions` instead of a seed.
- A variant with callbacks must handle `AppendMove`, and `held_karp` requires every variant component and dimension to handle `DPMove`. A variant that would be ignored now fails to compile instead.
- `ctx.tour()` and `ctx.position()` fail an assert during a replay and during `held_karp`, where they used to return an empty span and an unrelated tour.
- A dimension that commits state on `AppendMove` must provide staging (`begin_staging`, `discard_staging`, `save_staging`, `commit_staging`) to be used with `two_opt` or any replay, so that a rejected candidate leaves the committed tour's state intact.

### Fixed

- `variants/time_windows.hpp` and `variants/service_times.hpp` compile when included first: `RouteTiming` includes the `AppendMove` header it uses.
- The custom dimension example in VARIANTS.md read its position from `ctx.tour()`, which is empty during a replay, did not stage its state, and called an undefined `demand()`.
- `jonker_volgenant()`: forbidden edges cost `big_m` instead of the largest value of the cost type, which overflowed integer costs under `branch_and_bound` and directed `two_opt`.
- The README documented a tour cost function and variant arguments on algorithm methods that do not exist. A variant is set on the solver: `Solver(dist, variant)` or `set_variant`.

### Known issues

- `held_karp` keeps only the cheapest state per (visited set, city). Under time windows, a costlier partial tour that arrives earlier can hold the best completion, so `held_karp` can return a costlier tour as `optimal` (`Strict` with waiting, `Relaxed`) or report `infeasible` for a feasible instance (`Strict` with waiting). Pinned by `tests/algorithms/test_held_karp_time_windows.cpp`.

## v0.1.0 (2026-09-24)

### Library

- Header-only C++20 library, no external dependencies
- `DistanceSource` concept for plugging any distance model
- `DistanceMatrix`, `SymmetricDistanceMatrix`, `CoordinateDistance` built-in distance sources
- Distance functions: `euclidean`, `ceiling_euclidean`, `manhattan`, `chebyshev`, TSPLIB-specific (`euc_2d`, `ceil_2d`, `att`, `geo`)
- `Solver` with fluent chaining API and shared workspace buffers
- Neighbor lists (`solver.neighbors(city, k)`) for k-nearest lookups
- Runtime symmetry declaration (`set_symmetric`) with debug-mode matrix verification
- Variant callbacks (`move_prepare`, `move_filter`) and strategy callbacks (`select_next`) with partial tour support
- `CumulativeCost` dimension added automatically to every variant, so a replay can start from any prefix instead of the whole tour
- Evaluation primitives: `evaluate_append`, `evaluate_replay` / `accept_replay`, `evaluate_reversal` / `accept_reversal`, `prefix_cost`, `path_cost`, `save_staging` / `discard_staging`
- `rotate_to_front(city)`: rotates a complete tour at equal cost, for the symmetric mode where the first city is not preserved
- TSPTW variant (Strict and Relaxed modes) with multi-window time windows
- Greedy construction framework with pluggable strategy callbacks
- Jonker-Volgenant transformation for asymmetric TSP via symmetric-only algorithms
- TSPLIB95 parser (`load_tsplib`, `load_tsplib_coords`)
- CMake install support with `find_package(periple)` and `FetchContent`

### Algorithms

- Nearest neighbor (construction heuristic, O(n^2))
- Greedy construct (generic construction framework with pluggable strategy)
- Held-Karp (exact solver, O(n^2 * 2^n))
- 2-opt local search (`two_opt`): first improvement over neighbor lists, driven by a queue of active cities with don't-look bits. O(1) move evaluation without a variant, over the full cycle neighborhood when symmetry is declared and over non-wrapping segments otherwise; with a variant, candidates are replayed through the variant pipeline
- `nn_two_opt` registry entry (tag `2O`): nearest neighbor then 2-opt. Mean gap on Tier 2 + 3 instances: 6.2% against 24.7% for nearest neighbor alone
