# Periple

[![CI](https://github.com/jcailloux/periple/actions/workflows/ci.yml/badge.svg)](https://github.com/jcailloux/periple/actions/workflows/ci.yml)

An embeddable C++20 library for the Travelling Salesman Problem. Header-only, no dependencies. Use it as a standalone solver or as a component in delivery routing, warehouse picking, production scheduling, or any optimization system where the TSP is a subproblem.

Plug your data, pick an algorithm, read the tour. No framework, no boilerplate: Periple handles the infrastructure so you focus on your algorithm or your application.

## Quick start

From a TSPLIB benchmark instance:

```cpp
#include <periple/periple.hpp>
#include <periple/io/tsplib_parser.hpp>

auto dist = periple::load_tsplib("berlin52.tsp");
periple::Solver solver(dist);
solver.nearest_neighbor();

auto tour = solver.tour();  // std::span<const std::size_t>
auto cost = solver.cost();  // known optimum: 7542
```

From your own data:

```cpp
#include <periple/periple.hpp>
#include <periple/distance/coordinate.hpp>
#include <periple/distance/functions.hpp>

std::vector<std::pair<double, double>> sites = {{0, 0}, {3, 4}, {6, 1}, {2, 7}};
periple::CoordinateDistance dist(sites, periple::euclidean<int>(2));
periple::Solver solver(dist);
solver.nearest_neighbor();
```

## Features

- **Header-only**: drop the `include/` folder into your project
- **C++20**: concepts, spans, and modern type deduction
- **Fluent API**: chain algorithms with `solver.nearest_neighbor().two_opt()`
- **Modular**: include only the algorithm headers you need
- **Custom distance sources**: anything satisfying the `DistanceSource` concept works
- **TSPLIB95 support**: load standard benchmark instances out of the box
- **Zero-copy access**: `tour()` returns a `std::span`, no allocation
- **Efficient memory reuse**: shared buffers across algorithms, no redundant allocations

## Installation

Periple is header-only and requires a C++20 compiler.

<details>
<summary><b>Option 1: CMake FetchContent</b></summary>

```cmake
include(FetchContent)
FetchContent_Declare(
    periple
    GIT_REPOSITORY https://github.com/jcailloux/periple.git
    GIT_TAG        main
)
FetchContent_MakeAvailable(periple)

target_link_libraries(your_target PRIVATE periple::periple)
```

</details>

<details>
<summary><b>Option 2: CMake subdirectory</b></summary>

```cmake
add_subdirectory(path/to/periple)
target_link_libraries(your_target PRIVATE periple::periple)
```

</details>

<details>
<summary><b>Option 3: Copy headers</b></summary>

Copy the `include/periple/` directory into your project and add it to your include path.

</details>

## TSPLIB file loading

```cpp
#include <periple/io/tsplib_parser.hpp>

auto dist = periple::load_tsplib("berlin52.tsp");
periple::Solver solver(dist);
solver.nearest_neighbor();
```

`load_tsplib` returns a precomputed `SymmetricDistanceMatrix<int>` with O(1) lookups. It handles all supported TSPLIB formats: EUC_2D, GEO, ATT, CEIL_2D, EXPLICIT (LOWER_DIAG_ROW, UPPER_ROW, FULL_MATRIX).

> [!TIP]
> For very large instances (N > 10000) where the O(N^2) matrix would exceed available memory, `load_tsplib_coords` returns a `CoordinateDistance` that computes distances on the fly with O(N) storage:
> ```cpp
> auto dist = periple::load_tsplib_coords("pla85900.tsp");  // 85900 cities, ~1.4 MB
> ```
> Same `dist(i, j)` interface, same `Solver` compatibility -- only the memory/speed tradeoff differs.

## Solving

```cpp
periple::Solver solver(dist);

solver.nearest_neighbor({.start_city = 2});
auto tour = solver.tour();
auto cost = solver.cost();

solver.held_karp();              // replaces the tour with the optimal one
auto status = solver.status();   // SolutionStatus::optimal
```

Algorithm methods return `Solver&` for chaining:

```cpp
solver.nearest_neighbor().two_opt();
```

### Variant callbacks

Customize algorithm behavior without modifying internal logic. Variant callbacks are optional and zero-overhead when not used.

```cpp
#include <periple/variants/tsptw.hpp>

// TSPTW: reject moves that violate time windows
periple::tsptw::Strict tw(dist, windows);
solver.nearest_neighbor(tw);

// Custom construction strategy
solver.greedy_construct(MyStrategy{});
```

See [CALLBACKS.md](CALLBACKS.md) for the callback architecture and [VARIANTS.md](VARIANTS.md) for pre-built variants (TSPTW, etc.).

### Available algorithms

| Algorithm | Category | Complexity | ATSP support |
|-----------|----------|------------|--------------|
| Nearest neighbor | Construction | O(n^2) | full |
| Held-Karp | Exact | O(n^2 * 2^n) | full |

ATSP support: **full** = native asymmetric support, **adapted** = supported with different characteristics, **not yet** = symmetric only (use `jonker_volgenant()` to wrap your matrix).

### Generic frameworks

These are building blocks that require a user-supplied strategy. Complexity and ATSP correctness depend on the strategy. See [CALLBACKS.md](CALLBACKS.md).

| Framework | Category | Complexity with O(1) strategy |
|-----------|----------|-------------------------------|
| Greedy construct | Construction | O(n) |

> [!WARNING]
> Per-move variant callbacks defined for an ATSP instance are not compatible with the symmetric problem obtained via `jonker_volgenant()`.

### Selective includes

Include only the algorithms you use:

```cpp
#include <periple/core/solver.hpp>
#include <periple/algorithms/nearest_neighbor.hpp>
```

Or include everything: `#include <periple/periple.hpp>`

## Write your own algorithm

Periple handles distance computation, instance loading, and solution management so you can focus on your algorithm.

> [!NOTE]
> You do not need to contribute to the library to benefit from it. Use periple as infrastructure for your own research or application.

### Standalone usage

Load a TSPLIB instance and use `dist(i, j)` as a black box:

```cpp
#include <periple/io/tsplib_parser.hpp>

auto dist = periple::load_tsplib("berlin52.tsp");
std::size_t n = dist.size();

// --- Your algorithm here ---
std::vector<std::size_t> tour(n);
std::iota(tour.begin(), tour.end(), 0);
// ... build or improve the tour using dist(i, j) ...

// Compute tour cost
int cost = 0;
for (std::size_t i = 0; i < n; ++i)
    cost += dist(tour[i], tour[(i + 1) % n]);
```

For large instances, use `load_tsplib_coords` instead (O(N) memory, same `dist(i, j)` interface).

### Compare against built-in algorithms

Use the `Solver` to get a baseline, then compare:

```cpp
#include <periple/periple.hpp>
#include <periple/io/tsplib_parser.hpp>

auto dist = periple::load_tsplib("kroA100.tsp");
periple::Solver solver(dist);

// Periple's nearest neighbor as a baseline
solver.nearest_neighbor();
int nn_cost = solver.cost();

// Your algorithm
auto my_tour = my_algorithm(dist);
solver.set_tour(my_tour);
int my_cost = solver.cost();

std::printf("NN: %d  Mine: %d  Improvement: %.2f%%\n",
    nn_cost, my_cost,
    100.0 * (nn_cost - my_cost) / (double)nn_cost);
```

### Integrate into the library

If your algorithm works well, contributing it back makes it available to everyone and automatically benchmarks it against all others on TSPLIB instances. The step from standalone code to a contribution is small:

1. Move your logic into a `detail::` free function taking `std::span` buffers
2. Add a thin `Solver::` method wrapper
3. Register it in the benchmark runner (one line)

See [CONTRIBUTING.md](CONTRIBUTING.md) for the full guide and [BENCHMARKING.md](BENCHMARKING.md) for benchmark requirements.

## Distance sources

The `Solver` works with any type satisfying the `DistanceSource` concept. Periple ships three built-in sources:

| Source | Memory | Best for |
|---|---|---|
| `DistanceMatrix<T>` | O(N^2) | Small/medium instances, asymmetric distances |
| `SymmetricDistanceMatrix<T>` | O(N^2/2) | Symmetric instances (half the memory) |
| `CoordinateDistance` | O(N*D) | Large instances, on-the-fly computation |

Built-in distance functions (`euclidean`, `manhattan`, `chebyshev`, `ceiling_euclidean`) and TSPLIB-specific formulas are provided. Custom distance sources and custom distance functions (lambdas) are supported.

See [DISTANCE_SOURCES.md](DISTANCE_SOURCES.md) for all construction formats, distance functions, Jonker-Volgenant asymmetric TSP support, and custom distance source examples.

<details>
<summary><b>API reference</b></summary>

### Solver lifecycle

| Method | Description |
|--------|-------------|
| `Solver(dist)` | Create a solver bound to a distance source (template deduced) |
| `Solver(dist, tour_cost)` | Create with a custom tour cost function |
| `Solver()` | Create an empty solver (requires explicit template parameter) |
| `set_matrix(dist)` | Bind (or rebind) to a distance source, clearing any solution |
| `set_tour_cost(tc)` | Set or change the tour cost function |
| `set_symmetric(bool)` | Declare whether the problem is symmetric (default: `false`). Checked in debug |
| `set_symmetric(bool, unchecked)` | Same, but skips the debug symmetry check on the distance matrix |
| `clear()` | Clear the current solution, keep the distance source |
| `reset()` | Reset to default-constructed state |

### Solution state

| Method | Returns | Description |
|--------|---------|-------------|
| `status()` | `SolutionStatus` | `none`, `partial`, `feasible`, or `optimal` |
| `tour()` | `span<const city_type>` | Zero-copy view of the current tour |
| `cost()` | `cost_type` | Cost of the current tour |
| `size()` | `std::size_t` | Number of cities |
| `symmetric()` | `bool` | Whether the problem is declared symmetric |
| `set_tour(span)` | `void` | Inject a tour (full or partial prefix) |

### Algorithms

All algorithm methods return `Solver&` for chaining.

| Method | Description |
|--------|-------------|
| `nearest_neighbor(params)` | Construction heuristic |
| `nearest_neighbor(variant, params)` | With variant callbacks |
| `held_karp(params)` | Exact solver |
| `held_karp(variant, params)` | With variant callbacks (`DPMove`: filter, eval, on_improve) |
| `greedy_construct(strategy, params)` | Generic construction framework |
| `greedy_construct(strategy, variant, params)` | With variant callbacks forwarded to strategy |

</details>

## Benchmarking

All algorithms are benchmarked on TSPLIB95 instances with known optimal solutions. See [BENCHMARKING.md](BENCHMARKING.md).

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines on adding or improving algorithms.

## License

[MIT](LICENSE)