# Distance sources

Periple is generic over distance sources. Any type satisfying the `DistanceSource` concept works with the `Solver` and all algorithms. The library ships several built-in sources, and you can provide your own. The solver, like `JonkerVolgenantView`, keeps a pointer to its source, which must outlive it.

A tour's cost adds up in the source's `cost_type`, penalties included, so that type must hold the cost of any tour: for large integer costs, take `std::int64_t`.

## Matrix storage

Precomputed distance matrices with O(1) lookups. Best for small to medium instances.

```cpp
#include <periple/distance/matrix.hpp>

// Full square matrix (supports asymmetric distances)
periple::DistanceMatrix<int> matrix(n, {/* n*n values */});

// Declared symmetric by a trailing tag, which every constructor takes (see Symmetry)
periple::DistanceMatrix<int> declared(n, {/* n*n values */}, periple::symmetric);

// Symmetric matrix (half the memory)
periple::SymmetricDistanceMatrix<int> sym(n);
sym.set(0, 1, 10);
sym.set(0, 2, 15);
```

Both can also be built from a `std::vector<std::vector<T>>`, and `DistanceMatrix` from a flat `std::vector<T>`:

```cpp
periple::DistanceMatrix<int> matrix({{0, 10, 15}, {10, 0, 35}, {15, 35, 0}});
periple::SymmetricDistanceMatrix<int> sym({{0, 10, 15}, {10, 0, 35}, {15, 35, 0}});

std::vector<int> flat = compute_distances();
periple::DistanceMatrix<int> matrix2(n, std::move(flat));
```

## Coordinate-based (on-the-fly)

`CoordinateDistance` stores N points of D coordinates and computes distances on the fly. Memory is O(N*D) instead of O(N^2), making it suitable for large instances.

```cpp
#include <periple/distance/coordinate.hpp>
#include <periple/distance/functions.hpp>

std::vector<std::pair<double, double>> cities = {{0, 0}, {3, 4}, {6, 0}};
periple::CoordinateDistance cd(cities, periple::euclidean<int>(2));

periple::Solver solver(cd);
solver.nearest_neighbor();
```

### Construction formats

```cpp
CoordinateDistance cd(pairs, euclidean<int>(2));            // vector<pair>
CoordinateDistance cd(arrays, euclidean<double>(3));        // vector<array<double, D>>
CoordinateDistance cd(tuples, manhattan<int>(3));           // vector<tuple<double, ...>>
CoordinateDistance cd(vecs, chebyshev<int>(dim));           // vector<vector<double>>
CoordinateDistance cd(dim, flat_coords, my_dist_fn);       // flat vector + explicit dim
```

`CostT` is deduced from the distance function's return type via CTAD. For example, `euclidean<double>(2)` produces a `CoordinateDistance` with `cost_type = double`, while `euclidean<int>(2)` produces one with `cost_type = int`.

> [!IMPORTANT]
> Each call to `dist(i, j)` recomputes the distance from coordinates. For algorithms that make many distance queries, such as `two_opt`, a precomputed matrix is significantly faster (~1-3 ns lookup vs ~15-25 ns computation for 2D Euclidean). Use `CoordinateDistance` when the matrix would not fit in memory, or when distance computation is cheap relative to the algorithm.

## Built-in distance functions

### Generic factories

N-dimensional, templated on `CostT`. Each takes a dimension and returns a callable compatible with `CoordinateDistance`.

| Factory | Formula | Integral rounding |
|---|---|---|
| `euclidean<CostT>(dim)` | sqrt(sum (a_i - b_i)^2) | `round` (nearest) |
| `ceiling_euclidean<CostT>(dim)` | sqrt(sum (a_i - b_i)^2) | `ceil` |
| `manhattan<CostT>(dim)` | sum \|a_i - b_i\| | `round` (nearest) |
| `chebyshev<CostT>(dim)` | max \|a_i - b_i\| | `round` (nearest) |

For floating-point `CostT` (double, float), `ceiling_euclidean` returns the ceiling, and the others the exact value.

### TSPLIB95-specific functions

These implement the exact formulas from the [TSPLIB95 specification](http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/), including its specific constants and rounding rules. Always 2D, always `int`.

| Function | TSPLIB type | Notes |
|---|---|---|
| `tsplib::euc_2d(a, b)` | EUC_2D | nint(L2 norm) |
| `tsplib::ceil_2d(a, b)` | CEIL_2D | ceil(L2 norm) |
| `tsplib::att(a, b)` | ATT | ceil(sqrt(sum/10)), pseudo-Euclidean |
| `tsplib::geo(a, b)` | GEO | Great-circle, PI=3.141592, R=6378.388 |

### Custom distance functions

Any callable with signature `CostT fn(const double* a, const double* b)` works:

```cpp
auto weighted_l1 = [](const double* a, const double* b) -> double {
    return std::abs(a[0] - b[0]) + 2.0 * std::abs(a[1] - b[1]);
};
periple::CoordinateDistance cd(cities, weighted_l1);
```

To declare its symmetry, see [Symmetry](#symmetry).

## Asymmetric TSP (Jonker-Volgenant)

`JonkerVolgenantView` wraps any asymmetric N-city distance source as a symmetric 2N-city one, so that an algorithm written for symmetric distances, such as one of your own, solves ATSP instances. The built-in algorithms take asymmetric distances directly ([Available algorithms](README.md#available-algorithms)).

```cpp
#include <periple/distance/jonker_volgenant.hpp>

periple::DistanceMatrix<int> asym(n, {/* asymmetric */});

// Auto-computes big-M by scanning the matrix (O(n^2))
auto jv = periple::jonker_volgenant(asym);

// Or provide your own big-M to skip the scan
auto jv = periple::jonker_volgenant(asym, my_big_m);

// Solve as usual -- the Solver sees a symmetric 2n-city problem
periple::Solver solver(jv);
solver.nearest_neighbor();

// Convert back to the original n-city ATSP space
auto atsp_tour = jv.atsp_tour(solver.tour());   // extract n-city tour
auto atsp_cost = jv.atsp_cost(solver.cost());    // recover original cost
```

The transformation roughly quadruples solving time (2N cities instead of N) and may degrade heuristic solution quality. It is the standard approach used by LKH.

## Custom distance source

Any type satisfying the `DistanceSource` concept works with the `Solver`:

```cpp
struct MyDistance {
    using cost_type = double;
    using city_type = uint16_t;

    cost_type operator()(city_type i, city_type j) const;
    std::size_t size() const;
    static constexpr bool symmetric() { return true; }  // optional, see Symmetry
};

MyDistance dist{/* ... */};
periple::Solver solver(dist);  // dist must outlive the solver
solver.nearest_neighbor();
```

To adapt a third-party type that does not expose `cost_type` / `city_type` as nested members, specialize `dist_traits`:

```cpp
template <>
struct periple::dist_traits<ThirdPartyMatrix> {
    using cost_type = float;
    using city_type = int;
};
```

## Symmetry

A source declares with `symmetric()` that d(i, j) == d(j, i) for every pair: `static constexpr` when its type guarantees it, a member function when its data decide. A source without one is asymmetric. The solver takes the declaration when it receives the source. On symmetric distances, `two_opt` and `branch_and_bound` take faster paths: an undeclared symmetry costs speed only, a false one gives wrong results.

| Source | Symmetric |
|---|---|
| `load_tsplib(path)` | unless the file's `TYPE` is `ATSP` |
| `DistanceMatrix m(n, values)` | no |
| `DistanceMatrix m(n, values, periple::symmetric)` | yes |
| `SymmetricDistanceMatrix`, `jonker_volgenant(m)` | yes, by its type |
| `CoordinateDistance` with `euclidean`, `ceiling_euclidean`, `manhattan` or `chebyshev`; `load_tsplib_coords(path)` | yes, by its type |
| `CoordinateDistance` with any other function, `tsplib::` ones included | what the function object's `static constexpr bool symmetric()` returns; no for a lambda or a plain function |
| Custom source | what its `symmetric()` returns |

`solver.set_symmetric(bool)` overrides the declaration until the next `set_matrix`: for a source that cannot declare it, such as `CoordinateDistance` with a lambda or a third-party type adapted by `dist_traits`, or to run the directed `two_opt` on symmetric data.

In debug builds, the solver checks every pair when the declaration rests on the data: a member `symmetric()`, such as a tagged `DistanceMatrix`, or `set_symmetric(true)`. `set_symmetric(true, periple::unchecked)` skips the check, and a type that guarantees symmetry needs none. The check runs when the solver receives the source: after writing to a matrix a solver holds, pass it again with `set_matrix`.