# Contributing

## Build & test

```bash
cmake -B .build/debug -DCMAKE_BUILD_TYPE=Debug -G Ninja
cmake --build .build/debug
ctest --test-dir .build/debug --output-on-failure
```

Run a single test:

```bash
ctest --test-dir .build/debug -R algorithms --output-on-failure
```

CI runs automatically on every push and pull request to `main` and `dev` (Debug + Release). All tests must pass before merging.

Pull requests should target the `dev` branch. `main` is updated from `dev` at release time.

## Architecture

### Solver

`Solver<Dist, Variant>` is the central type. `Dist` is the distance source, `Variant` is the variant (defaults to `NoCallbacks`). CTAD allows `Solver solver(matrix)` or `Solver solver(matrix, variant)`.

The Solver stores a pointer to the variant (`const Variant*`). The variant must outlive the Solver.

### Evaluation pipeline

Every algorithm uses the same pipeline to evaluate candidate moves:

```
1. ctx_.init(move, dist, tour, position, cost)   -- initialize dimension state
2. invoke_prepare(variant, move, ctx_)            -- variant adjusts cost_delta/dimensions
3. invoke_filter(variant, move, ctx_)             -- variant checks feasibility
4. score = dist(from, to) + ctx_.cost_delta       -- final score
```

Key Solver methods:

- **`evaluate_append(city)`** -- runs the pipeline, returns `std::optional<double>` (nullopt if filtered). Does not modify observable state (writes to `mutable ctx_`).
- **`append(city)`** -- runs init + prepare, applies the move (places city, updates cost), commits dimension state.
- **`rebuild_and_cost(tour)`** -- replays a complete tour through the pipeline (init + prepare + commit for each city), reconstructing dimension state and cost. Used by `set_tour`, `resume_at`, `clear`.
- **`close_tour()`** -- adds the return-to-start edge cost for complete tours.
- **`evaluate_replay(city_at, from, prefix)`** / **`evaluate_reversal(i, j)`** -- score a rewrite of an existing tour by replaying the changed suffix through the pipeline. Used by local search, see [CALLBACKS.md](CALLBACKS.md).

### EvalContext

The Solver owns a `context_type` deduced from the variant's `dimension` typedef:

| Variant | Context |
|---|---|
| No `dimension` typedef | `EmptyContext<CityT, CostT>` |
| `using dimension = T` | `EvalContext<CityT, CostT, T>` |
| `using dimension = Dimensions<Ts...>` | `EvalContext<CityT, CostT, Ts...>` |

`EvalContext` provides:
- `cost_delta` (double, mutable) -- additive cost adjustment
- `dim<D>()` -- access dimension D
- `tour()`, `position()`, `cost()` -- read-only solver state
- `init()`, `commit()`, `snapshot()`, `restore()` -- lifecycle methods

### Algorithm structure

Algorithms are defined inline in `algorithms/*.hpp`. Each algorithm header includes `core/solver.hpp` and defines a `Solver::method()`.

Constructive algorithms use `greedy_construct` with a strategy:

```cpp
// NearestSelector calls solver.evaluate_append() in a loop, picks the minimum.
template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::nearest_neighbor(NearestNeighborParams params) -> Solver& {
    return greedy_construct(NearestSelector{}, ConstructParams{...});
}
```

`greedy_construct` handles the generic loop: start city, repeated `strategy.select_next(solver)` + `append`, termination on `nullopt`.

Exact algorithms (Held-Karp) use the pipeline directly with `DPMove`.

> [!IMPORTANT]
> `position_` is the inverse index of `tour_` and `visited_` its membership set. Both are invariants: every primitive that writes `tour_` maintains them, so an algorithm never refreshes them. Write the tour through those primitives rather than touching `tour_` directly.

## Where algorithm state lives

| Kind | Examples | Lifecycle |
|---|---|---|
| Tour invariant | `position_`, `visited_` | always consistent with `tour_`; every tour mutation maintains it |
| Matrix-derived | `neighbors_`, `hk_cache_` | lazy, persistent, invalidated by `set_matrix` |
| Tour-derived, versioned | `path_fwd_`/`path_bwd_` | lazy via `ensure_path_costs()`, validated against `tour_version_`, maintained incrementally by `rebuild_path_costs(from)` during a run |
| Dimensions | `RouteTiming`, `CumulativeCost` | declared by variants, maintained eagerly by the pipeline |
| Algorithm workspace | `dont_look_`, `queue_` | refreshed on entry by the algorithm that uses them, maintained during its run, never read afterwards |

Three rules follow:

- `tour_` and `n_` are written only by the primitives (`append`, `set_tour`, `held_karp`, `accept_replay`, `accept_reversal`, `reverse_range`, `reverse_cyclic`), each of which increments `tour_version_`. A cache derived from the tour compares its own version against it, so a missing increment leaves that cache marked valid for a tour it no longer describes.
- An algorithm never reads a workspace buffer it did not refresh in the same call.
- Public accessors never expose the workspace category.

Local search primitives available for reuse: `detail::ActiveQueue` (FIFO of active cities with don't-look bits), `reverse_range` / `reverse_cyclic`, `ensure_path_costs` / `rebuild_path_costs`, and the public `prefix_cost` / `evaluate_reversal` / `accept_reversal` / `path_cost`.

### Output invariants

Every method leaves the solver in a consistent state. The registry test loops check these around each algorithm call through `SolverTestAccess` (`tests/solver_test_access.hpp`, a friend under `PERIPLE_TESTING`), so a registered algorithm inherits the whole checklist without writing a test:

| Invariant | Checked by |
|---|---|
| `position_` consistent with `tour_` | `position_consistent()` |
| `visited_` consistent with `tour_` | `visited_consistent()` |
| `cost_` equal to a pipeline recomputation | `cost == recompute`, or self-consistency through `set_tour` |
| `status_` consistent with `n_` | feasible/optimal implies `n_ == size()` |
| path costs up to date or marked stale | `path_costs_consistent()` |
| no staging left active | `staging_inactive()` |
| `tour_version_` changed whenever the tour changed | `run_checked(solver, call)` |

An algorithm that can stop early must satisfy them at the point where it stops.

## Adding an algorithm

The steps below use Or-opt as a running example. `algorithms/two_opt.hpp` is a worked example of the same path, for a local search that reuses the Solver's local search primitives.

### Step 1: Parameter struct

In `core/solver.hpp`, add a parameter struct with sensible defaults:

```cpp
struct OrOptParams {
    std::size_t neighbors = 10;  // candidate list size per city (0 = all cities)
    std::size_t max_moves = 0;   // stop after this many applied moves (0 = no limit)
};
```

### Step 2: Define Move types (only if needed)

Only if the existing move types do not describe what your algorithm evaluates. Move types are part of the public interface and determine which callback overloads users can provide; each lives in its own header under `core/moves/`:

```cpp
// core/moves/or_opt_move.hpp
template <typename CityT>
struct OrOptMove {
    CityT city;                    // first city of the moved segment
    std::size_t from, len, to;     // source position, length, destination
};
```

A new move type requires adding overloads to existing variants. Existing variant updates will be handled during review -- open your PR with the algorithm and the maintainer will handle variant updates if needed.

2-opt needed none: it scores a candidate tour by replaying the changed suffix as a sequence of `AppendMove`, so every existing variant works with it unchanged. Prefer that over a new move type unless an O(1) evaluation per dimension is the point of the algorithm.

### Step 3: Declare the Solver method

In the `Solver` class in `core/solver.hpp`:

```cpp
auto or_opt(OrOptParams params = {}) -> Solver&;
```

### Step 4: Implement the algorithm header

Create `algorithms/or_opt.hpp`. Start with a reference block citing academic sources. Use the evaluation pipeline:

```cpp
#pragma once

// Or-opt local search
//
// Or (1976), "Traveling Salesman-Type Combinatorial Problems and Their
//   Relation to the Logistics of Regional Blood Banking"

#include <periple/core/solver.hpp>

namespace periple {

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::or_opt(OrOptParams params) -> Solver& {
    // Construction: evaluate_append(), append(), ctx_.init(), invoke_prepare().
    // Improvement: evaluate_replay() / accept_replay() to score and apply a
    //   rewrite of the current tour through the variant pipeline.
    // Set status_ to SolutionStatus::feasible for heuristics,
    //   SolutionStatus::optimal for exact solvers.
}

} // namespace periple
```

### Step 5: Add to the umbrella header

In `periple/periple.hpp`:

```cpp
#include <periple/algorithms/or_opt.hpp>
```

### Step 6: Register in the algorithm registry

In `algorithms/registry.hpp`, add a descriptor struct and append it to the `AllAlgorithms` tuple:

```cpp
struct AlgoNNOrOpt {
    static constexpr const char* tag  = "OO";
    static constexpr const char* name = "nn_or_opt";
    static constexpr bool is_exact          = false;
    static constexpr bool symmetric_only    = false;
    static constexpr bool is_metaheuristic  = false;
    static constexpr int  max_tier          = 5;

    template <DistanceSource Dist, typename V>
    void operator()(Solver<Dist, V>& s, unsigned = 0) const {
        s.nearest_neighbor();
        if (s.status() == SolutionStatus::feasible) s.or_opt();
    }
};

using AllAlgorithms = std::tuple<AlgoNearestNeighbor, AlgoHeldKarp, AlgoNNTwoOpt, AlgoNNOrOpt>;
```

The registry runs each entry on a fresh solver, so an improvement heuristic has to construct a tour first. Guard the improvement call: under a hard constraint the construction can stop early, leaving nothing to improve.

This automatically enables the algorithm in `test_algorithms` (contract tests), `test_variants` (variant cross-product tests), and the [benchmark runner](BENCHMARKING.md).

### Step 7: Verify

```bash
cmake --build .build/debug
ctest --test-dir .build/debug -R "algorithms|variants" --output-on-failure
```

<details>
<summary><b>Optional: Add an algorithm-specific cache</b></summary>

If the algorithm needs persistent state across calls (DP tables, populations, etc.), follow these three steps:

**a) Create a cache file** in `core/caches/`. The struct is templated on `CostT`/`CityT` and provides a `reset()` method:

```cpp
// core/caches/lk_cache.hpp
#pragma once

#include <vector>

namespace periple {

template <typename CostT, typename CityT>
struct LKCache {
    std::vector<CostT> alpha;
    // ...

    void reset() {
        alpha.clear();
    }
};

} // namespace periple
```

**b) Register the cache in `solver.hpp`**: include the header, add an `std::optional` member, and add the `reset()` call in `invalidate_caches()`:

```cpp
#include <periple/core/caches/lk_cache.hpp>

std::optional<LKCache<cost_type, city_type>> lk_cache_;

// In invalidate_caches()
if (lk_cache_) lk_cache_->reset();
```

**c) Initialize lazily** in the `Solver::` method:

```cpp
if (!lk_cache_) lk_cache_.emplace();
```

`invalidate_caches()` is called by `set_matrix()` when the solver is bound to a new problem. This ensures stale data from a previous problem is never reused.

Shared workspace buffers (`tour_`, `position_`, `visited_`, `dont_look_`, `neighbors_`) are grow-only and reused across algorithms. Prefer using those over allocating new buffers. Only add a cache for data that is specific to your algorithm and expensive to recompute.

</details>

## Adding a variant

Variants encode problem-specific constraints. A variant provides `move_prepare` and/or `move_filter` overloads for each move type it supports.

### Step 1: Create the variant header

Create `variants/my_constraint.hpp` in `include/periple/variants/`. Provide overloads for each supported move type:

```cpp
#pragma once

#include <periple/core/dimensions/route_timing.hpp>

namespace periple::my_constraint {

struct Strict {
    using dimension = RouteTiming;  // declares which dimensions this variant needs

    // Constructive algorithms (AppendMove)
    template <typename CityT, typename Ctx>
    void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const { /* ... */ }

    // Exact algorithms (DPMove)
    template <typename CityT, typename Ctx>
    void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    bool move_filter(const DPMove<CityT>& m, const Ctx& ctx) const { /* ... */ }
};

} // namespace periple::my_constraint
```

### Step 2: Write tests

Add focused tests in `tests/` for specific behaviors. Add the variant to `tests/algorithms/test_variants.cpp` for cross-algorithm testing.

### Step 3: Document

Add the variant to [CALLBACKS.md](CALLBACKS.md) and [VARIANTS.md](VARIANTS.md) with usage examples.

See `variants/time_windows.hpp` and `variants/service_times.hpp` for complete examples.

## Writing tests

Tests use plain `assert()` with descriptive messages. Core tests live in `tests/`. Algorithm tests are in `tests/algorithms/`.

```
tests/
    CMakeLists.txt
    test_distance_matrix.cpp
    test_solver.cpp
    test_callbacks.cpp
    test_composed.cpp
    test_replay.cpp
    solver_test_access.hpp
    algorithms/
        CMakeLists.txt
        test_algorithms.cpp
        test_variants.cpp
        test_two_opt.cpp
```

### Automated algorithm tests

Any algorithm registered in the [algorithm registry](#step-6-register-in-the-algorithm-registry) is automatically tested by `test_algorithms`. For each algorithm, the runner checks:

1. **Tour validity** -- every city appears exactly once (sizes 0 through 5).
2. **Cost consistency** -- reported cost matches the recomputed sum of edge weights.
3. **Optimality** -- exact algorithms must match the brute-force optimum; heuristics must be >= optimum.
4. **Symmetric instances** -- tested with `SymmetricDistanceMatrix`.
5. **Asymmetric instances** -- tested with `DistanceMatrix`.

Run a single algorithm's tests by passing its tag:

```bash
.build/debug/tests/algorithms/test_algorithms NN
.build/debug/tests/algorithms/test_algorithms --help   # list available tags
```

### Algorithm-specific tests

The automated suite covers the common contract. If your algorithm has specific behavior worth testing (improvement invariants, parameter variations, etc.), add a dedicated test file in `tests/algorithms/` and register it in `tests/algorithms/CMakeLists.txt`.

## Performance guidelines

- Reason about whether an optimization actually helps before adding it. An extra branch in a tight loop to handle a rare case is often worse than just handling it uniformly.
- Don't add indirection (virtual calls, `std::function`, extra pointer chases) unless there is a clear design reason.
- Document the expected time complexity.
- Use the shared workspace buffers (`visited_`, `position_`, etc.) rather than allocating temporaries.
- Profile on realistic instance sizes before drawing conclusions. A micro-benchmark on 10 cities says little about behavior on 1000.

## Naming conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Solver method | `algorithm_name()` (snake_case) | `nearest_neighbor()`, `two_opt()`, `held_karp()` |
| Param struct | `AlgorithmNameParams` (PascalCase) | `NearestNeighborParams`, `TwoOptParams` |
| Cache struct | `AbbreviationCache` (PascalCase) | `HKCache`, `LKCache`, `GACache` |
| Cache file | `abbreviation_cache.hpp` | `hk_cache.hpp`, `lk_cache.hpp` |
| Algorithm header | `algorithm_name.hpp` | `greedy_construct.hpp`, `two_opt.hpp` |
| Registry struct | `AlgoAlgorithmName` (PascalCase) | `AlgoNearestNeighbor`, `AlgoNNTwoOpt` |
| Variant namespace | `snake_case` | `time_windows`, `service_times` |
| Variant struct | `PascalCase` | `Strict`, `Relaxed`, `ServiceTimes` |
| Dimension struct | `PascalCase` | `RouteTiming`, `RouteLoad` |

If cache abbreviations ever conflict, use a longer form (e.g. `OrOptCache` instead of `OOCache`).

## Benchmark requirements

Every new algorithm must be [registered in the algorithm registry](#step-6-register-in-the-algorithm-registry). The benchmark runner picks it up automatically (see [BENCHMARKING.md](BENCHMARKING.md)). Include results in the PR description:

**Constructive heuristic**: run on Tier 2 + 3. Report mean gap, max gap, and time.

**Improvement heuristic** (2-opt, Or-opt, LK, ...): same as constructive, plus before/after comparison on the same instances showing the improvement delta.

**Exact solver**: must find the known optimum on every Tier 1 instance. Report solving times.

**Metaheuristic**: 10 runs with seeds 42-51. Report best, mean, and stddev.

## Code conventions

- Comments in English.
- No external dependencies. Standard library only.
- Use `std::span` for buffer parameters.
- Prefer `static_cast` over C-style casts.
- All `assert()` must include descriptive messages.
- Match the existing code style (naming, indentation, const-correctness).
- Every algorithm header must start with a reference block:

    ```cpp
    // Lin-Kernighan heuristic
    //
    // Lin, Kernighan (1973), "An Effective Heuristic Algorithm for the
    //   Traveling-Salesman Problem"
    // Helsgaun (2000), "An Effective Implementation of the Lin-Kernighan
    //   Traveling Salesman Heuristic" - added 5-opt moves and alpha-nearness
    // @jdupont (2026) - SIMD acceleration for neighbor lookup
    ```
