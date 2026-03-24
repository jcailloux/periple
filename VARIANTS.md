# Variants

Variants encode problem-specific constraints as callback structs. They plug into the [callback architecture](CALLBACKS.md) without modifying algorithm internals.

Each variant provides two modes:
- **Strict**: hard constraints via `move_filter` (reject infeasible moves)
- **Relaxed**: soft constraints via `tour_cost` (penalize violations in the objective)

## TSPTW: Time Windows

```cpp
#include <periple/variants/tsptw.hpp>
```

Each city has one or more time windows `{earliest, latest}`. A vehicle arriving before `earliest` waits; arriving after `latest` is a violation.

### TimeWindow

```cpp
periple::tsptw::TimeWindow windows[] = {
    {0, 100},   // city 0: depot, no constraint
    {0,  50},   // city 1: must arrive by time 50
    {10, 80},   // city 2: service between 10 and 80
};
```

For cities with multiple windows (e.g., a shop closed at noon):

```cpp
std::vector<std::vector<periple::tsptw::TimeWindow>> windows = {
    {{0, 100}},            // city 0: always open
    {{8, 12}, {14, 18}},   // city 1: closed 12-14
    {{0, 100}},            // city 2: always open
};
```

Windows are sorted internally by `earliest`. An arrival between two windows waits for the next one to open.

### Strict

Rejects infeasible moves via `move_filter`. Maintains an arrival time cache via `on_commit`.

```cpp
periple::tsptw::Strict tw(matrix, windows);
periple::Solver solver(matrix);
solver.nearest_neighbor({}, tw);
```

The algorithm skips cities whose time window would be violated. If all candidates are rejected at a given step, a fallback picks the first unvisited city (the instance may be infeasible for this construction order).

Strict uses `move_filter` and `on_commit` for `AppendMove`. It does not define `tour_cost`, `move_eval`, or `move_score`.

### Relaxed

Penalizes time window violations in the tour cost. No move rejection.

```cpp
periple::tsptw::Relaxed tw(matrix, windows, /*penalty_weight=*/1000);
periple::Solver solver(matrix, tw);     // tour_cost includes penalties
solver.nearest_neighbor();              // selection uses distance (no per-move callbacks)
```

`solver.cost()` returns the total distance plus `penalty_weight * violation` for each city where the arrival exceeds the latest window. When multiple windows exist, the violation is the minimum lateness across all windows.

Relaxed is passed as the `TourCost` template parameter of the Solver (second argument to the constructor). It does not define per-move callbacks.

### Choosing penalty_weight

There is no universal value. The weight should be calibrated to the ratio between distance costs and violation amplitudes in your instance:
- Distances in [1, 100], violations in [0, 10]: a weight of 100 may suffice
- Distances in [1, 10000], violations in [0, 1]: a weight of 100000+ is needed

The default of 1000 is reasonable for typical TSPLIB-scale instances. Too low and the solver ignores windows; too high and it over-prioritizes feasibility at the expense of distance.

### Combining Strict and Relaxed

Strict and Relaxed address different use cases. They can also be combined: use Strict as per-move callbacks to reject clearly infeasible moves, and Relaxed as `tour_cost` to penalize borderline violations in the reported cost. This requires two separate instances since they serve different roles.

```cpp
periple::tsptw::Strict strict_tw(matrix, windows);
periple::tsptw::Relaxed relaxed_tw(matrix, windows, 1000);
periple::Solver solver(matrix, relaxed_tw);     // tour_cost with penalties
solver.nearest_neighbor({}, strict_tw);          // move_filter rejects infeasible
```

## Writing your own variant

A variant is a struct with callback methods detected via `if constexpr`. Implement only the callbacks you need:

```cpp
struct MyVariant {
    // Reject infeasible moves (hard constraint).
    bool move_filter(std::span<const std::size_t> tour,
                     const periple::AppendMove<std::size_t>& m) const {
        // return false to reject
    }

    // Update caches after a city is placed.
    void on_commit(std::span<const std::size_t> tour,
                   const periple::AppendMove<std::size_t>& m) const {
        // update internal state
    }
};
```

For a variant usable as `tour_cost`, add `operator()`:

```cpp
struct MyTourCost {
    template <periple::DistanceSource Dist>
    auto operator()(const Dist& dist,
                    std::span<const typename periple::dist_traits<Dist>::city_type> tour) const
        -> typename periple::dist_traits<Dist>::cost_type
    {
        // return total cost
    }
};
```

See [CALLBACKS.md](CALLBACKS.md) for the full callback reference (the 5 callbacks, move types, selectors, score type deduction).
