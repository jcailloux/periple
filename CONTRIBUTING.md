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

Every algorithm follows a two-layer pattern inside a single header file (`algorithms/<name>.hpp`):

**Layer 1: `detail::` free function.** Pure algorithmic logic. Takes its inputs and output buffers as `std::span`. No dependency on `Solver`. This function is independently testable and benchmarkable.

```cpp
namespace periple {
namespace detail {

template <DistanceSource Dist>
auto nearest_neighbor_build(
    const Dist& dist, std::size_t n,
    typename dist_traits<Dist>::city_type start,
    std::span<typename dist_traits<Dist>::city_type> tour,      // output
    std::span<uint8_t> visited)                                 // workspace
    -> typename dist_traits<Dist>::cost_type;

} // namespace detail
} // namespace periple
```

**Layer 2: `Solver::` method.** A thin wrapper declared in `core/solver.hpp` and defined `inline` in the algorithm header. It sizes workspace buffers, calls the `detail::` function, updates solver state, and returns `*this` for chaining.

```cpp
template <DistanceSource Dist>
inline auto Solver<Dist>::nearest_neighbor(NearestNeighborParams params)
    -> Solver&
{
    n_ = dist_->size();
    ensure_shared(n_);
    cost_ = detail::nearest_neighbor_build(
        *dist_, n_,
        static_cast<city_type>(params.start_city),
        std::span<city_type>(tour_.data(), n_),
        std::span<uint8_t>(visited_.data(), n_));
    status_ = SolutionStatus::feasible;
    rebuild_position();
    return *this;
}
```

Key points:

- The `detail::` function never touches `Solver` internals. It receives everything it needs through its parameters.
- The `Solver::` method handles buffer management and state updates only. No algorithmic logic.
- Set `status_` to `SolutionStatus::feasible` for heuristics, `SolutionStatus::optimal` for exact solvers.

> [!IMPORTANT]
> `position_` is the inverse index of `tour_`: `position_[city]` gives the position of a city in the current tour. Improvement algorithms use it for O(1) lookups. If your algorithm modifies the tour and reads `position_`, call `rebuild_position()` each time the tour changes and `position_` is needed. At minimum, always call it once at the end of the `Solver::` method so that subsequent algorithms in a chain see a consistent state.

## Adding an algorithm

### Step 1: Parameter struct

In `core/solver.hpp`, add a parameter struct with sensible defaults:

```cpp
struct TwoOptParams {
    std::size_t max_iterations = 0;  // 0 = no limit
};
```

### Step 2: Declare the Solver method

In the `Solver` class in `core/solver.hpp`:

```cpp
auto two_opt(TwoOptParams params = {}) -> Solver&;
```

### Step 3: Implement the algorithm header

Create `algorithms/two_opt.hpp`. Start the file with a reference block documenting the academic sources and any subsequent improvements:

```cpp
#pragma once

// 2-opt local search
//
// Croes (1958), "A Method for Solving Traveling-Salesman Problems"
// @contributor (2026) -description of a non-paper-based improvement

#include <periple/core/solver.hpp>

namespace periple {
namespace detail {

template <DistanceSource Dist>
auto two_opt_improve(
    const Dist& dist, std::size_t n,
    std::span<typename dist_traits<Dist>::city_type> tour,
    std::span<typename dist_traits<Dist>::city_type> position,
    typename dist_traits<Dist>::cost_type current_cost,
    TwoOptParams params)
    -> typename dist_traits<Dist>::cost_type
{
    // Pure algorithm logic here
}

} // namespace detail

template <DistanceSource Dist>
inline auto Solver<Dist>::two_opt(TwoOptParams params) -> Solver&
{
    n_ = dist_->size();
    ensure_shared(n_);
    cost_ = detail::two_opt_improve(
        *dist_, n_,
        std::span<city_type>(tour_.data(), n_),
        std::span<city_type>(position_.data(), n_),
        cost_, params);
    status_ = SolutionStatus::feasible;
    rebuild_position();
    return *this;
}

} // namespace periple
```

### Step 4: Add to the umbrella header

In `periple/periple.hpp`:

```cpp
#include <periple/algorithms/two_opt.hpp>
```

### Step 5: Register in the algorithm registry

In `core/registry.hpp`, add a descriptor struct and append it to the `AllAlgorithms` tuple:

```cpp
struct AlgoTwoOpt {
    static constexpr const char* tag  = "2O";
    static constexpr const char* name = "two_opt";
    static constexpr bool is_exact       = false;
    static constexpr bool symmetric_only = false;
    static constexpr int  max_tier       = 5;

    template <DistanceSource Dist>
    void operator()(Solver<Dist>& s) const { s.two_opt(); }
};

using AllAlgorithms = std::tuple<AlgoNearestNeighbor, AlgoHeldKarp, AlgoTwoOpt>;
```

This single registration automatically enables the algorithm in both the [unified test suite](#writing-tests) and the [benchmark runner](BENCHMARKING.md).

### Step 6: Verify

Run `test_algorithms` to confirm the new algorithm passes all automated checks:

```bash
cmake --build .build/debug
.build/debug/tests/algorithms/test_algorithms 2O
```

<details>
<summary><b>Step 6: Add an algorithm-specific cache (if needed)</b></summary>

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

`reset()` should clear data that is tied to a specific problem instance. The algorithm will resize buffers as needed on the next call.

**b) Register the cache in `solver.hpp`**: include the header, add an `std::optional` member, and add the `reset()` call in `invalidate_caches()`:

```cpp
// Include
#include <periple/core/caches/lk_cache.hpp>

// Member (in the Solver class)
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

## Writing tests

Tests use plain `assert()` with no test framework. Core tests (distance matrices, solver, JonkerVolgenant) live in `tests/`. Algorithm tests are handled by a unified runner in `tests/algorithms/test_algorithms.cpp`:

```
tests/
├── CMakeLists.txt
├── test_distance_matrix.cpp
├── test_solver.cpp
└── algorithms/
    ├── CMakeLists.txt
    └── test_algorithms.cpp
```

### Automated algorithm tests

Any algorithm registered in the [algorithm registry](#step-5-register-in-the-algorithm-registry) is automatically tested by `test_algorithms`. For each algorithm, the runner checks:

1. **Tour validity** -- every city appears exactly once (sizes 0 through 5).
2. **Cost consistency** -- reported cost matches the recomputed sum of edge weights.
3. **Optimality** -- exact algorithms must match the brute-force optimum; heuristics must be >= optimum.
4. **Symmetric instances** -- tested with `SymmetricDistanceMatrix`.
5. **Asymmetric instances** -- tested with `DistanceMatrix` (or via JonkerVolgenant for `symmetric_only` algorithms).

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
- Document the expected time complexity in the `detail::` function.
- Use the shared workspace buffers (`visited_`, `position_`, etc.) rather than allocating temporaries.
- Profile on realistic instance sizes before drawing conclusions. A micro-benchmark on 10 cities says little about behavior on 1000.

## Naming conventions

| Element | Convention | Example |
|---------|-----------|---------|
| Solver method | `algorithm_name()` (snake_case) | `nearest_neighbor()`, `two_opt()`, `held_karp()` |
| Detail function | `detail::algorithm_name_verb()` | `detail::nearest_neighbor_build()`, `detail::two_opt_improve()`, `detail::held_karp_solve()` |
| Param struct | `AlgorithmNameParams` (PascalCase) | `NearestNeighborParams`, `TwoOptParams` |
| Cache struct | `AbbreviationCache` (PascalCase) | `HKCache`, `LKCache`, `GACache` |
| Cache file | `abbreviation_cache.hpp` | `hk_cache.hpp`, `lk_cache.hpp` |
| Algorithm header | `algorithm_name.hpp` | `nearest_neighbor.hpp`, `two_opt.hpp` |
| Registry struct | `AlgoAlgorithmName` (PascalCase) | `AlgoNearestNeighbor`, `AlgoTwoOpt` |

Detail function verbs by category:

| Category | Verb | Example |
|----------|------|---------|
| Construction | `_build` | `detail::nearest_neighbor_build()` |
| Local search | `_improve` | `detail::two_opt_improve()` |
| Exact | `_solve` | `detail::held_karp_solve()` |
| Metaheuristic | `_search` | `detail::simulated_annealing_search()` |

If cache abbreviations ever conflict, use a longer form (e.g. `OrOptCache` instead of `OOCache`).

## Benchmark requirements

Every new algorithm must be [registered in the algorithm registry](#step-5-register-in-the-algorithm-registry). The benchmark runner picks it up automatically (see [BENCHMARKING.md](BENCHMARKING.md)). Include results in the PR description:

**Constructive heuristic**: run on Tier 2 + 3. Report mean gap, max gap, and time.

**Improvement heuristic** (2-opt, Or-opt, LK, ...): same as constructive, plus before/after comparison on the same instances showing the improvement delta.

**Exact solver**: must find the known optimum on every Tier 1 instance. Report solving times.

**Metaheuristic**: 10 runs with seeds 42-51. Report best, mean, and stddev.

## Code conventions

- Comments in English.
- No external dependencies. Standard library only.
- Use `std::span` for buffer parameters in `detail::` functions.
- Prefer `static_cast` over C-style casts.
- Match the existing code style (naming, indentation, const-correctness).
- Every algorithm header must start with a reference block. Cite the original paper and any subsequent improvements, whether from a paper or a contributor:

    ```cpp
    // Lin-Kernighan heuristic
    //
    // Lin, Kernighan (1973), "An Effective Heuristic Algorithm for the
    //   Traveling-Salesman Problem"
    // Helsgaun (2000), "An Effective Implementation of the Lin-Kernighan
    //   Traveling Salesman Heuristic" - added 5-opt moves and alpha-nearness
    // @jdupont (2026) - SIMD acceleration for neighbor lookup
    ```
