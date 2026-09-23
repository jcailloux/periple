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
- **`finalize_closing_edge()`** -- adds the return-to-start edge cost for complete tours.

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
> `position_` is the inverse index of `tour_`: `position_[city]` gives the position of a city in the current tour. If your algorithm modifies the tour and reads `position_`, call `rebuild_position()` each time the tour changes and `position_` is needed.

## Adding an algorithm

### Step 1: Parameter struct

In `core/solver.hpp`, add a parameter struct with sensible defaults:

```cpp
struct TwoOptParams {
    std::size_t max_iterations = 0;  // 0 = no limit
};
```

### Step 2: Define Move types

Each algorithm family defines its own Move types. These are part of the public interface and determine which callback overloads users can provide. Define them in `core/moves.hpp`:

```cpp
struct TwoOptMove { std::size_t i, j; };
```

A new move type requires adding overloads to existing variants. Existing variant updates will be handled during review -- open your PR with the algorithm and the maintainer will handle variant updates if needed.

### Step 3: Declare the Solver method

In the `Solver` class in `core/solver.hpp`:

```cpp
auto two_opt(TwoOptParams params = {}) -> Solver&;
```

### Step 4: Implement the algorithm header

Create `algorithms/two_opt.hpp`. Start with a reference block citing academic sources. Use the evaluation pipeline:

```cpp
#pragma once

// 2-opt local search
//
// Croes (1958), "A Method for Solving Traveling-Salesman Problems"

#include <periple/core/solver.hpp>

namespace periple {

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::two_opt(TwoOptParams params) -> Solver& {
    // Use evaluate_append(), append(), ctx_.init(), invoke_prepare(), invoke_filter()
    // Set status_ to SolutionStatus::feasible for heuristics,
    //   SolutionStatus::optimal for exact solvers.
}

} // namespace periple
```

### Step 5: Add to the umbrella header

In `periple/periple.hpp`:

```cpp
#include <periple/algorithms/two_opt.hpp>
```

### Step 6: Register in the algorithm registry

In `algorithms/registry.hpp`, add a descriptor struct and append it to the `AllAlgorithms` tuple:

```cpp
struct AlgoTwoOpt {
    static constexpr const char* tag  = "2O";
    static constexpr const char* name = "two_opt";
    static constexpr bool is_exact          = false;
    static constexpr bool symmetric_only    = false;
    static constexpr bool is_metaheuristic  = false;
    static constexpr int  max_tier          = 5;

    template <DistanceSource Dist, typename V>
    void operator()(Solver<Dist, V>& s, unsigned = 0) const { s.two_opt(); }
};

using AllAlgorithms = std::tuple<AlgoNearestNeighbor, AlgoHeldKarp, AlgoTwoOpt>;
```

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
    algorithms/
        CMakeLists.txt
        test_algorithms.cpp
        test_variants.cpp
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
| Registry struct | `AlgoAlgorithmName` (PascalCase) | `AlgoNearestNeighbor`, `AlgoTwoOpt` |
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
