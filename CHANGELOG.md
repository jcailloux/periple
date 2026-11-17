# Changelog

## Unreleased

### Added

- `time_limit` in the params of every algorithm: stopped early, it returns its best complete tour (`feasible`) or else a partial one (`partial`).
- Benchmark runner: `--time-limit MS`.
- `select_next(solver, stop)`: a construction strategy that runs long can honor `time_limit`.
- `branch_and_bound()`: exact depth-first search through the `AppendMove` pipeline, so it honors every variant and stays exact under time windows. A feasible tour held by the solver is the first to beat, `resume_at` searches the completions of a prefix, `max_nodes` and `time_limit` stop it early.
- `branch_and_bound(bound)`: the same search with a lower bound strategy (`lower_bound`, optional `begin`). Built-in bounds: `SpanningTreeBound` for symmetric distances and `SpanningArborescenceBound` for any instance, Held-Karp Lagrangian relaxations ascended at every node and the defaults of `branch_and_bound()`; `AssignmentBound`, for costs without structure under time windows.
- `branch_and_bound(bound, order)`: the order of the children is a strategy too, through `order` (once per node, permutes the scored children) or `pick` (before each descent, the next child), or both. `CheapestFirst` is the default. `dead_end` is what a lower bound returns for a node without a feasible completion.
- `time_windows::DeadlineBound(bound)`: a bound for `branch_and_bound` under `Strict` windows, around another bound. It prunes a node from which an unvisited city, or the return to the first, can no longer meet its deadline, even by the shortest path. `branch_and_bound()` applies it under a single `Strict`, alone or composed only with `ServiceTimes` and `Relaxed`.
- `Strict::windows(city)` and `Relaxed::windows(city)`: a city's windows, sorted by `earliest`.
- `variant()` returns the solver's variant and `Composed::component<I>()` the I-th composed variant, so that a strategy can read them.
- A distance source declares its symmetry with `symmetric()`, and the solver takes that declaration; `set_symmetric` overrides it. `SymmetricDistanceMatrix`, `jonker_volgenant()`, `CoordinateDistance` with a generic metric and the TSPLIB loaders declare it.
- `DistanceMatrix` takes a trailing `periple::symmetric` to declare it symmetric, checked in debug builds when a solver receives it.
- `held_karp(engine)`: the engine of the dynamic program. `periple::AllSets` holds every visited set, up to 25 cities; `periple::ReachableSets` only the sets that moves accepted by the variant's `DPMove` filter reach, up to 64 cities; `periple::ReachableLabels` the sets of `ReachableSets`, with every partial tour per set and last city that no other dominates. `held_karp()` takes `ReachableLabels` when a dimension of the variant keeps DP state that the cheapest partial tour may not settle, as under time windows, else `ReachableSets` when the variant filters `DPMove`, else `AllSets`. `AllSets` and `ReachableSets` passed where `held_karp()` takes `ReachableLabels` report `optimal` and `infeasible` only when proven, and `feasible`, or `partial` without a tour, otherwise.

### Changed

- `held_karp()` without a variant: about a fifth of the memory with `int` costs from 19 cities, and up to a fifth less time on a fresh solver.
- `held_karp()` under a variant that filters `DPMove` creates only the visited sets that accepted moves reach: 10 to 170 times less time when the filter prunes most sets, up to 1.6 times more when it prunes little ([Filter strength](CALLBACKS.md#filter-strength)). `held_karp(periple::AllSets{})` keeps the table.
- `held_karp()` under `Strict` takes `ReachableLabels`, and each engine drops the partial tours that can no longer meet a deadline, as `branch_and_bound()` does. Under windows that never make the vehicle wait, where `held_karp` was already exact, it takes at most 60% of the former memory, and from about a ten-thousandth of the former time where deadlines prune most orders to 1.5 times where they prune none. `held_karp(periple::AllSets{})` stays `optimal` there, within 15% of the former time where no deadline binds, and up to 19 times faster under deadlines.

### Breaking

- Registry descriptors derive from `Algorithm<Derived>` and define `run(solver, RunOptions)`, plus `setup(solver)` for an improvement algorithm; `operator()` takes `RunOptions` instead of a seed.
- A variant with callbacks must handle `AppendMove`, and `held_karp` requires every variant component and dimension to handle `DPMove`. A variant that would be ignored now fails to compile instead.
- `ctx.tour()` and `ctx.position()` fail an assert during a replay and during `held_karp`, where they used to return an empty span and an unrelated tour.
- A dimension that commits state on `AppendMove` must provide staging (`begin_staging`, `discard_staging`, `save_staging`, `commit_staging`) to be used with `two_opt` or any replay, so that a rejected candidate leaves the committed tour's state intact.
- `load_tsplib` returns a `DistanceMatrix<int>`, declared symmetric unless the file's `TYPE` is `ATSP`, instead of a `SymmetricDistanceMatrix<int>`: about twice the memory. Code that names the type changes; `auto` does not.
- `two_opt` takes its symmetric mode on every source declared symmetric, where it needed `set_symmetric(true)`: the tour found can differ, its first city too (`rotate_to_front` puts it back). `set_symmetric(false)` keeps the directed search.
- `set_matrix` replaces a `set_symmetric` override with the new source's declaration, where it kept the override. `set_symmetric` requires a source, asserted in debug builds: call it after `set_matrix`.
- `set_variant` clears the solution, as `set_matrix` does, where it kept the tour with the cost and status computed under the former variant. To start from the tour, copy it and pass it to `set_tour` after `set_variant`.
- `ServiceTimes` requires nonnegative service times, asserted in debug builds.
- A dimension that commits `DPMove` state provides `resize_dp(states)`, `begin_dp(state)` and `compare_dp(state)`, and stores its state at `DPMove::from_state` and `to_state` instead of indexing by `set`. `RouteTiming::dp_n_` is removed.

### Fixed

- `variants/time_windows.hpp` and `variants/service_times.hpp` compile when included first: `RouteTiming` includes the `AppendMove` header it uses.
- The custom dimension example in VARIANTS.md read its position from `ctx.tour()`, which is empty during a replay, did not stage its state, and called an undefined `demand()`.
- `jonker_volgenant()`: forbidden edges cost `big_m` instead of the largest value of the cost type, which overflowed integer costs under `branch_and_bound` and directed `two_opt`.
- The README documented a tour cost function and variant arguments on algorithm methods that do not exist. A variant is set on the solver: `Solver(dist, variant)` or `set_variant`.
- `load_tsplib` reads a `FULL_MATRIX` whole: it kept the lower triangle, so an ATSP instance loaded wrong, without error. Under another `TYPE`, asymmetric `FULL_MATRIX` data throw `std::runtime_error` instead of losing their upper triangle.
- `held_karp` under `Relaxed`, and under `Strict` windows that make the vehicle wait, kept only the cheapest partial tour per visited set and last city: it could return a costlier tour as `optimal`, or `infeasible` for a feasible instance. `held_karp()` takes `ReachableLabels` there.
- `held_karp` started from the first city at time 0, without its service time or the wait for its window, which `append()` and every other algorithm apply: it could return as `optimal` a costlier tour or one that misses a window.
- `held_karp` under a `RouteTiming` variant, after `set_matrix` to a larger instance, indexed past the end of its DP departures.
- `set_tour` and `resume_at` held a tour as `feasible` or `partial` even when `move_filter` rejects one of its moves, and `nearest_neighbor` resumed from a rejected prefix could return as `feasible` a tour that violates the variant. Both now replay the tour through `move_filter` as a construction does and hold a rejected tour as `infeasible`; resumed from one, `nearest_neighbor` and `greedy_construct` return at once.
- `nearest_neighbor` and `greedy_construct` with `resume_at` equal to `size()` left the complete tour `partial`.

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
