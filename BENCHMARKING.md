# Benchmarking

All algorithms in periple are benchmarked on [TSPLIB95](http://comopt.ifi.uni-heidelberg.de/software/TSPLIB95/) instances with known optimal solutions. Results are directly comparable to any published TSP paper since 1991.

## Running

```bash
cmake -B .build/release -DCMAKE_BUILD_TYPE=Release -DPERIPLE_BUILD_BENCHMARKS=ON -G Ninja
cmake --build .build/release
.build/release/benchmarks/periple_bench                  # all algorithms, all tiers
.build/release/benchmarks/periple_bench --tier 1,2       # tiers 1-2 only
.build/release/benchmarks/periple_bench --algo NN        # single algorithm
.build/release/benchmarks/periple_bench --algo NN,HK     # multiple algorithms
.build/release/benchmarks/periple_bench --help            # list available options and algorithm tags
.build/release/benchmarks/periple_bench --output results.json
```

## Instances

31 TSPLIB instances committed to `benchmarks/instances/`, organized in five tiers:

| Tier | Size      | Instances | Purpose |
|------|-----------|-----------|---------|
| 1    | N <= 25   | burma14, ulysses16, gr17, gr21, ulysses22, gr24 | Exact algorithm verification |
| 2    | N <= 200  | att48, berlin52, st70, eil76, kroA100, kroB100, ch130, ch150, kroA200 | Core heuristic quality |
| 3    | N <= 1000 | a280, lin318, pcb442, att532, rat575, rat783 | Speed discrimination |
| 4    | N > 1000  | pr1002, u1817, pr2392, pcb3038, fnl4461, rl5934 | Scalability |
| 5    | N > 10000 | usa13509, d18512, pla33810, pla85900 | Stress (manual only) |

Tier 1 includes three `EXPLICIT` matrix instances (gr17, gr21, gr24) to exercise all supported TSPLIB formats. Known optima are stored in `benchmarks/optima.json`.

## What is measured

**Gap to optimum** -- the primary metric:

```
gap(%) = (solution_cost - optimum) / optimum * 100
```

**Tour validation** -- every solution is checked before comparison:
1. Tour size equals N.
2. Every city appears exactly once (valid Hamiltonian cycle).
3. Reported cost matches the recomputed sum of edge weights.

An invalid tour fails the benchmark for that algorithm/instance pair.

**Wall-clock time** -- measured end-to-end (excluding I/O). For solves under 10 ms, repeated and reported as the median. Reported only, never a pass/fail criterion.

**Peak RSS** -- memory high-water mark. Reported for comparison, not enforced.

## Exact vs heuristic

| | Exact | Constructive / Improvement heuristic | Metaheuristic |
|---|---|---|---|
| **Quality check** | Must match known optimum | Gap reported | Gap reported (best, mean, stddev) |
| **Failure condition** | Optimum not found on Tier 1 | None (comparison only) | None (comparison only) |
| **Runs** | 1 | 1 | 10, seeds 42-51 |
| **Tier range** | 1 only | 2-5 | 2-5 |

## Output

The runner produces:
- **Stdout tables** -- per-algorithm results and a cross-algorithm comparison.
- **JSON** -- one file per run with all metrics, suitable for automated analysis.

```
nearest_neighbor
Instance       N    Optimum     Cost   Gap(%)  Time(ms)  Valid  Status
berlin52      52       7542     8965    18.87      0.02  yes    feasible
kroA100      100      21282    24310    14.22      0.04  yes    feasible
```

## Adding an algorithm to the benchmark

Register it in the shared algorithm registry (`include/periple/core/registry.hpp`). Both the benchmark runner and the test suite pick it up automatically. See [CONTRIBUTING.md](CONTRIBUTING.md) for the full checklist and what benchmark results to include in a PR.

## Symmetric TSP only

All benchmarks target the symmetric TSP (d(i,j) = d(j,i)). periple handles asymmetric instances via the Jonker-Volgenant transformation (N cities -> 2N symmetric cities), but the inherent overhead of this transformation would add noise to comparison tables rather than insight.
