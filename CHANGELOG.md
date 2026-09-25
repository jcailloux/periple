# Changelog

## Unreleased

### Breaking

- A variant with callbacks must handle `AppendMove`, and `held_karp` requires every variant component and dimension to handle `DPMove`. A variant that would be ignored now fails to compile instead.
- `ctx.tour()` and `ctx.position()` fail an assert during a replay and during `held_karp`, where they used to return an empty span and an unrelated tour.
- A dimension that commits state on `AppendMove` must provide staging (`begin_staging`, `discard_staging`, `save_staging`, `commit_staging`) to be used with `two_opt` or any replay, so that a rejected candidate leaves the committed tour's state intact.

### Fixed

- The custom dimension example in VARIANTS.md read its position from `ctx.tour()`, which is empty during a replay, did not stage its state, and called an undefined `demand()`.

### Known issues

- `held_karp` under time windows with waiting (`earliest > 0`) keeps only the cheapest state per (visited set, city) and can report `infeasible` for a feasible instance. Pinned by `tests/algorithms/test_held_karp_waiting_windows.cpp`.

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
