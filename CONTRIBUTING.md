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

## Use of generative AI

AI assistants are welcome. Say in the pull request which one you used and for what. You answer for what you submit as for code written by hand: you have read, understood and tested it, and checked every citation it makes against its source.

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
- **`pop_back(prefix_cost)`** -- undoes the last `append`: frees the city and restores the given cost. Dimensions need no undo, the next `append` overwrites the freed position.
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
- `tour()`, `position()` -- the placed prefix and its inverse index, during construction (asserted)
- `cost()` -- the cost so far
- `init()`, `commit()`, `snapshot()`, `restore()` -- lifecycle methods
- `begin_staging()`, `discard_staging()`, `save_staging()`, `commit_staging()` -- replay staging, forwarded to the dimensions that provide it

### Algorithm structure

Algorithms are defined inline in `algorithms/*.hpp`, one header per `Solver::method()`, which includes `core/solver.hpp`. A ready-made strategy has its own header in the directory of the framework whose contract it implements, and includes that framework: `NearestSelector` in `algorithms/greedy_construct/nearest_selector.hpp`, the bounds of `branch_and_bound` in `algorithms/branch_and_bound/`, next to its search (`search.hpp`). A contract adopted by a second framework moves, with its strategies, to a directory of its own. A method's header includes the strategies it uses by default; a framework's own default, such as `CheapestFirst`, is defined with it.

Constructive algorithms use `greedy_construct` with a strategy:

```cpp
// NearestSelector calls solver.evaluate_append() in a loop, picks the minimum.
template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::nearest_neighbor(NearestNeighborParams params) -> Solver& {
    return greedy_construct(NearestSelector{}, ConstructParams{...});
}
```

`greedy_construct` handles the generic loop: start city, repeated `strategy.select_next(solver)` + `append`, termination on `nullopt`.

`held_karp` uses the pipeline directly with `DPMove`. `branch_and_bound` extends and backtracks through `evaluate_append`, `append` and `pop_back`, and takes as strategies its lower bound and, optionally, the order of the children; `branch_and_bound()` picks a built-in bound.

> [!IMPORTANT]
> `position_` is the inverse index of `tour_` and `visited_` its membership set. Both are invariants: every primitive that writes `tour_` maintains them, so an algorithm never refreshes them. Write the tour through those primitives rather than touching `tour_` directly.

## Where algorithm state lives

| Kind | Examples | Lifecycle |
|---|---|---|
| Tour invariant | `position_`, `visited_` | always consistent with `tour_`; every tour mutation maintains it |
| Matrix-derived | `neighbors_`, `hk_cache_` | lazy, persistent, invalidated by `set_matrix` |
| Tour-derived, versioned | `path_fwd_`/`path_bwd_` | lazy via `ensure_path_costs()`, validated against `tour_version_`, maintained incrementally by `rebuild_path_costs(from)` during a run |
| Dimensions | `RouteTiming`, `CumulativeCost` | declared by variants, maintained eagerly by the pipeline |
| Algorithm workspace | `dont_look_`, `queue_`, `bb_cache_` | refreshed on entry by the algorithm that uses them, maintained during its run, never read afterwards |

Three rules follow:

- Every write to `tour_` or `n_` increments `tour_version_`. Write through the existing primitives (`append`, `pop_back`, `set_tour`, `accept_replay`, `accept_reversal`, `reverse_range`, `reverse_cyclic`), which do it for you; an algorithm that fills the tour itself, as `held_karp` does, increments it itself. A cache derived from the tour compares its own version against it, so a missing increment leaves that cache marked valid for a tour it no longer describes.
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

### Variant and strategy support

Every algorithm honors every TSP variant:

- **Variant callbacks.** Every move the algorithm evaluates or applies goes through the variant pipeline: `ctx_.init`, `invoke_prepare`, `invoke_filter`, then `ctx_.commit` for an applied move. The primitives `evaluate_append` / `append`, `evaluate_replay` / `accept_replay` and `evaluate_reversal` / `accept_reversal` do this for you. With a variant set, every decision and every cost comes from the pipeline. Callbacks read `ctx.tour()` and `ctx.position()` during construction only.
- **Move types.** Prefer an existing move type, see [Step 2](#step-2-define-move-types-only-if-needed). `Solver` checks `AppendMove` coverage itself, and `evaluate_replay` checks staging. For each other move type the algorithm emits, add the `static_assert`s that the variant and its dimensions handle it, as `held_karp` does for `DPMove`.
- **Strategy callbacks.** If a framework exists for the algorithm's family, the algorithm is a strategy of it: `nearest_neighbor()` is `greedy_construct(NearestSelector{})`. A framework is introduced when a second algorithm shares the same skeleton. An exact algorithm takes a strategy only where the choice leaves the result exact, as the lower bound and the order of the children of `branch_and_bound`, with a facade for the built-in choice.
- **Justified exceptions.** A method that genuinely cannot honor a variant, because it is impossible or unreasonably costly, rejects it with a `static_assert` giving the reason, as `rotate_to_front` and `path_cost` do. State the reason in its declaration comment, list the case in the "Applicability" table of [CALLBACKS.md](CALLBACKS.md), and add a [compile-fail test](#compile-fail-tests). Each variant is thus either honored or rejected at compile time.

### Time limit support

Every algorithm honors a time limit, as it honors every variant: `time_limit` in its [params](#step-1-parameter-struct), forwarded by its [descriptor](#step-6-register-in-the-algorithm-registry). `test_limits` checks the rules below.

- **Polling.** Write the body as a private template on the deadline type, called through `detail::with_deadline(params.time_limit, ...)` (`core/deadline.hpp`). Each loop opens its own `detail::Poller stop(deadline);`, whose stride adapts to that loop's units, and tests `stop()`. Without a limit, no clock read is compiled.
- **Grain.** Poll before each unit of work, preparation included (filling a table, building a cache). A unit spans about ten elementary operations at least, and 2n variant callbacks or O(n) distance evaluations at most. A strategy call that can exceed this takes `stop` as its last argument.
- **Stop.** Once expired, leave within n + 1 variant callbacks on the best complete tour (`feasible`), or else a valid partial one (`partial`): what a construction built, the prefix a search started from.
- **Tolerance.** A call limited to T returns within T + 50 µs + one unit + the stop, preemption aside, while consecutive units cost within a factor of 2.
- **Caches.** A stop can leave a cache that outlives the call half-built: the next call resumes it, so that a limit below its build time still progresses. Its invariant test compares every entry declared ready with a fresh computation, after an interrupted build, after its resumption and after `set_matrix`. A wrong entry only degrades a tour, which no other test sees.

### Step 1: Parameter struct

In `core/solver.hpp`, add a parameter struct with sensible defaults:

```cpp
struct OrOptParams {
    std::size_t neighbors = 10;  // candidate list size per city (0 = all cities)
    std::size_t max_moves = 0;   // stop after this many applied moves (0 = no limit)
    std::chrono::nanoseconds time_limit{0};  // stop after this much wall time (0 = no limit)
};
```

### Step 2: Define Move types (only if needed)

Add one when the existing move types cannot describe what your algorithm evaluates. Move types are part of the public interface and determine which callback overloads users can provide; each lives in its own header under `core/moves/`:

```cpp
// core/moves/or_opt_move.hpp
template <typename CityT>
struct OrOptMove {
    CityT city;                    // first city of the moved segment
    std::size_t from, len, to;     // source position, length, destination
};
```

A new move type needs:
- an entry in `detail::handles_any_move` (`core/dimensions.hpp`), so the compile-time checks cover it;
- overloads in every shipped variant that can honor it, in the shipped dimensions (`core/dimensions/`), in `LoggingCallbacks` and in the `EdgeSurcharge` and `ForbiddenEdges` oracles of `test_variants.cpp`;
- a row in the move types table of [CALLBACKS.md](CALLBACKS.md).

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
    // Time limit: see Time limit support.
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
struct AlgoNNOrOpt : Algorithm<AlgoNNOrOpt> {
    static constexpr const char* tag  = "OO";
    static constexpr const char* name = "nn_or_opt";
    static constexpr bool is_exact          = false;
    static constexpr bool symmetric_only    = false;
    static constexpr bool is_metaheuristic  = false;
    static constexpr int  max_tier          = 5;

    // The tour or_opt improves, built outside its time limit.
    template <DistanceSource Dist, typename V>
    void setup(Solver<Dist, V>& s) const { s.nearest_neighbor(); }

    template <DistanceSource Dist, typename V>
    void run(Solver<Dist, V>& s, const RunOptions& o) const {
        if (s.status() == SolutionStatus::feasible) s.or_opt({.time_limit = o.time_limit});
    }
};

using AllAlgorithms = std::tuple<AlgoNearestNeighbor, AlgoHeldKarp, AlgoNNTwoOpt, AlgoNNOrOpt>;
```

`run` is the algorithm under test and forwards `RunOptions`. `setup` builds the tour an improvement algorithm starts from, since the registry runs each entry on a fresh solver. Guard `run`: under a hard constraint the construction can stop early, leaving nothing to improve.

This enables the algorithm in the [automated tests](#automated-algorithm-tests) and the [benchmark runner](BENCHMARKING.md).

### Step 7: Verify

```bash
cmake --build .build/debug
ctest --test-dir .build/debug --output-on-failure
```

Then go through the [Definition of done](#definition-of-done).

### Definition of done

A PR adding an algorithm is complete when:

- the [automated tests](#automated-algorithm-tests) pass in Debug and Release;
- the algorithm follows [Variant and strategy support](#variant-and-strategy-support), and a new move type comes with everything listed in [Step 2](#step-2-define-move-types-only-if-needed);
- the algorithm follows [Time limit support](#time-limit-support);
- `tests/test_callbacks.cpp` has a protocol test using `LoggingCallbacks`: exact counts and order of `move_prepare` / `move_filter` on a small instance, for each move type the algorithm emits (see `protocol_held_karp_logging`);
- [algorithm-specific tests](#algorithm-specific-tests) cover any behavior beyond the common contract;
- removing the algorithm's `invoke_filter` calls, then its `invoke_prepare` calls, then its `ctx_.commit` calls makes at least one test fail each time; an algorithm built on the pipeline primitives (`evaluate_append`, `append`, `evaluate_replay`) passes the same check on its use of them, for example by scoring on raw distances or by ignoring the status `append` leaves. Check it once by hand and revert: it proves the tests see the variant;
- `README.md` ("Available algorithms"), `CALLBACKS.md` ("Applicability") and `CHANGELOG.md` (`## Unreleased`, public interface changes under `### Breaking`) are updated, and a new test file is added to the tree in [Writing tests](#writing-tests) and to `CLAUDE.md`;
- the PR description includes the [benchmark results](#benchmark-requirements).

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

    // Every algorithm but held_karp (AppendMove)
    template <typename CityT, typename Ctx>
    void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    bool move_filter(const AppendMove<CityT>& m, const Ctx& ctx) const { /* ... */ }

    // held_karp (DPMove)
    template <typename CityT, typename Ctx>
    void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    bool move_filter(const DPMove<CityT>& m, const Ctx& ctx) const { /* ... */ }
};

} // namespace periple::my_constraint
```

Provide callbacks for `AppendMove` and `DPMove`. A constraint that an exact DP cannot honor keeps `AppendMove` alone, with the reason in the header; `held_karp` then refuses to compile with the variant.

### Step 2: Write tests

Add an entry to `tests/algorithms/variant_registry.hpp`: `test_variants` then runs the variant under every registered algorithm, against the brute-force optimum of the `AppendMove` pipeline. `exact_optimal = false` marks a [known bug](#known-bugs). Add focused tests in `tests/` for behaviors specific to the variant.

### Step 3: Document

Add the variant to [CALLBACKS.md](CALLBACKS.md) and [VARIANTS.md](VARIANTS.md) with usage examples.

See `variants/time_windows.hpp` and `variants/service_times.hpp` for complete examples.

## Writing tests

Tests use plain `assert()` with descriptive messages. Core tests live in `tests/`. Algorithm tests are in `tests/algorithms/`. Asserts stay active in every build type: `tests/CMakeLists.txt` removes `NDEBUG`, so the Release job checks as much as the Debug one.

```
tests/
    CMakeLists.txt
    test_distance_matrix.cpp
    test_solver.cpp
    test_callbacks.cpp
    test_composed.cpp
    test_replay.cpp
    test_jonker_volgenant.cpp
    test_tsplib_parser.cpp
    logging_callbacks.hpp
    solver_test_access.hpp
    compile_fail/       one file per combination rejected at compile time
    death/              one program per misuse caught by an assert
    algorithms/
        CMakeLists.txt
        test_algorithms.cpp
        test_variants.cpp
        test_limits.cpp
        variant_registry.hpp
        test_two_opt.cpp
        test_held_karp_time_windows.cpp
        test_branch_and_bound.cpp
```

### Automated algorithm tests

Any algorithm registered in the [algorithm registry](#step-6-register-in-the-algorithm-registry) is tested automatically, its traits (`is_exact`, `symmetric_only`) narrowing or strengthening what is checked:

- **`test_algorithms`**: valid tour (sizes 0 to 5), reported cost equal to a recomputation, the brute-force optimum for an exact algorithm and at least the optimum for a heuristic, on symmetric and asymmetric instances, a second run after `set_matrix`, and the [output invariants](#output-invariants) around every call.
- **`test_variants`**: every shipped variant of `tests/algorithms/variant_registry.hpp`, against the brute-force optimum of the `AppendMove` pipeline; and oracles with a constrained optimum known by brute force, one per mechanism: a surcharge per edge (`move_prepare`), forbidden edges (`move_filter`), deadlines (a dimension). An exact algorithm must also prove infeasibility and drop the tour on instances with no tour.
- **`test_limits`**: [Time limit support](#time-limit-support) under an expired limit and a short one, preparation included, then a second run on the stopped solver.

Run a single algorithm's tests by passing its tag:

```bash
.build/debug/tests/algorithms/test_algorithms NN
.build/debug/tests/algorithms/test_algorithms --help   # list available tags
```

### Algorithm-specific tests

The automated suite covers the common contract. If your algorithm has specific behavior worth testing (improvement invariants, parameter variations, etc.), add a dedicated test file in `tests/algorithms/` and register it in `tests/algorithms/CMakeLists.txt`.

### Compile-fail tests

A combination rejected at compile time is tested in `tests/compile_fail/`, one file per case, registered with `periple_compile_fail(name "static_assert message")` in `tests/CMakeLists.txt`. The test passes when the build fails with that message.

### Death tests

A misuse caught by an assert is tested in `tests/death/`, one program per case, registered with `periple_death_test(name "assert message")` in `tests/CMakeLists.txt`. The test passes when the program aborts with that message.

### Known bugs

A known bug is pinned by a test that asserts the wrong result, in its own executable, with a comment saying so (see `test_held_karp_time_windows.cpp`). It fails once the bug is fixed: turn it into an ordinary test then. List the bug under `### Known issues` in `CHANGELOG.md`.

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
| Strategy header | `strategy_name.hpp`, in `algorithms/framework_name/` | `greedy_construct/nearest_selector.hpp`, `branch_and_bound/assignment_bound.hpp` |
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
