# Changelog

## v0.1.0 (2026-03-21)

Initial release.

### Library

- Header-only C++20 library, no external dependencies
- `DistanceSource` concept for plugging any distance model
- `DistanceMatrix`, `SymmetricDistanceMatrix`, `CoordinateDistance` built-in distance sources
- Distance functions: `euclidean`, `ceiling_euclidean`, `manhattan`, `chebyshev`, TSPLIB-specific (`euc_2d`, `ceil_2d`, `att`, `geo`)
- `Solver` with fluent chaining API and shared workspace buffers
- Jonker-Volgenant transformation for asymmetric TSP via symmetric-only algorithms
- TSPLIB95 parser (`load_tsplib`, `load_tsplib_coords`)
- CMake install support with `find_package(periple)` and `FetchContent`

### Algorithms

- Nearest neighbor (construction heuristic, O(n^2))
- Held-Karp (exact solver, O(n^2 * 2^n))