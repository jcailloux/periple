# Choosing a solver

The measurements behind [Choosing an exact solver](README.md#choosing-an-exact-solver), each on one core of an i5-13500.

## Exact solvers

`branch_and_bound` has no size limit and is exact under every variant with non-negative penalties. Without a variant it outruns `held_karp` beyond a dozen cities: 5 ms against 4.5 s on 24 random asymmetric cities. Its speed depends on its bound. By default, the bound fits the declared symmetry ([Symmetry](DISTANCE_SOURCES.md#symmetry)) and, under strict time windows, drops each partial tour that can no longer meet a deadline:

| Instance | Call | Measured |
|---|---|---|
| Symmetric | `branch_and_bound()` | Tier 1 within 3 ms, st70 in 1.6 s, kroB100 in 29 s, kroA100 unproved after 60 s |
| Symmetric, under strict time windows | `branch_and_bound()` | Every case measured, berlin52 and st70, within 0.48 s |
| Asymmetric | `branch_and_bound()` | TSPLIB: ftv38 in 84 ms, ry48p in 0.17 s, ft70 in 3.0 s; p43, ftv64 and ftv70 unproved after 60 s |
| Asymmetric, under strict time windows | `branch_and_bound()` | On berlin52 and st70, every case measured within 0.84 s but one, unproved after 20 s |
| Asymmetric, costs without structure, under strict time windows | `branch_and_bound(periple::time_windows::DeadlineBound(periple::AssignmentBound{}))` | On uniform random costs, every case measured within 0.18 s, where the arborescence takes up to 10 s. On berlin52 and st70, keep the arborescence: around `AssignmentBound`, 6 of the 24 cases stay unproved after 20 s |

A complete tour held by the solver is the first tour to beat, which cuts the search time by 12 to 61% on the Tier 2 instances proved. Started from a nearest-neighbor tour improved by 2-opt and stopped after 10 s, 8 of the 9 Tier 2 instances end proved or improved.

### Under strict time windows

The cases measured: the first 20, 30 and 40 cities of berlin52 and st70, each distance raised by up to 20% for the asymmetric ones, and asymmetric uniform random costs. Around the arrival times of a nearest-neighbor tour improved by 2-opt, each city but the first gets a window of 2, 5 or 10 mean arcs on either side, or so many that no deadline binds.

`held_karp()` applies the same deadline test, and its time and memory follow the partial tours that can still meet every deadline rather than the number of cities. On the same cases, against the calls of the table:

| Windows | `held_karp()` |
|---|---|
| 2 or 5 mean arcs on either side | Faster in 27 of the 29 cases both solve, 10 times faster at the median, each within 46 ms and 27 MB; solves in 3.1 ms the case that `branch_and_bound()` leaves unproved |
| 10 mean arcs on either side | Slower in 12 of the 14 cases both solve, by 2.6 to 1,100 times, each within 1.3 s and 730 MB; no tour after 20 s on the last, with about 12 GB |
| No binding deadline | 100 to 3,200 times slower at 20 cities, 0.36 to 0.40 s and 226 MB; no tour after 20 s at 30 and 40 cities, with 14 to 16 GB |

The deadline test applies under a single `Strict`, alone or composed only with `ServiceTimes` and `Relaxed`. With a variant of your own in the composition, both solvers leave it out: [CALLBACKS.md](CALLBACKS.md#branch_and_bound) says when to pass it to `branch_and_bound`.
