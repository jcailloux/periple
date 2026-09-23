# Variants

Variants encode problem-specific constraints as callback structs. They plug into the [callback architecture](CALLBACKS.md) without modifying algorithm internals.

## Service Times

```cpp
#include <periple/variants/service_times.hpp>
```

Adds per-city service durations to the departure time. Uses the `RouteTiming` dimension.

```cpp
double durations[] = {0, 5, 3, 2};  // service time at each city
periple::service_times::ServiceTimes svc(durations);
periple::Solver solver(matrix, svc);
solver.nearest_neighbor();
```

Service times do not affect the tour cost directly (cost is based on travel distances). They shift the departure time at each city, which matters when composed with time windows or other time-dependent variants.

## Time Windows (TSPTW)

```cpp
#include <periple/variants/time_windows.hpp>
```

Each city has one or more time windows `{earliest, latest}`. A vehicle arriving before `earliest` waits; arriving after `latest` is a violation.

### TimeWindow

```cpp
periple::time_windows::TimeWindow windows[] = {
    {0, 100},   // city 0: depot, no constraint
    {0,  50},   // city 1: must arrive by time 50
    {10, 80},   // city 2: service between 10 and 80
};
```

For cities with multiple windows (e.g., a shop closed at noon):

```cpp
std::vector<std::vector<periple::time_windows::TimeWindow>> windows = {
    {{0, 100}},            // city 0: always open
    {{8, 12}, {14, 18}},   // city 1: closed 12-14
    {{0, 100}},            // city 2: always open
};
```

Windows are sorted internally by `earliest`. An arrival between two windows waits for the next one to open.

For cities without constraints, use `std::optional<TimeWindow>`:

```cpp
std::optional<periple::time_windows::TimeWindow> windows[] = {
    std::nullopt,                                  // city 0: unconstrained
    periple::time_windows::TimeWindow{0, 50},      // city 1: constrained
    std::nullopt,                                  // city 2: unconstrained
};
```

### Strict

Hard constraint. Rejects infeasible moves via `move_filter`. Adjusts departure time via `move_prepare` (waiting for window open).

```cpp
periple::time_windows::Strict tw(windows);
periple::Solver solver(matrix, tw);
solver.nearest_neighbor();
```

If all candidates are rejected at a given step, construction stops (partial tour). The Strict variant uses `RouteTiming` to track arrival and departure times.

Local search applies the same filter. `two_opt` scores a candidate reversal by replaying the affected suffix, so a reversal that would violate a window is rejected and the tour stays feasible:

```cpp
solver.nearest_neighbor();
if (solver.status() == periple::SolutionStatus::feasible)
    solver.two_opt();
```

The guard is what makes the chain safe: under a hard constraint the construction can stop early, and there is then no complete tour to improve.

### Relaxed

Soft constraint. Penalizes time window violations via `move_prepare` (adds `penalty_weight * violation` to `cost_delta`). Never rejects moves.

```cpp
periple::time_windows::Relaxed relaxed(windows, /*penalty_weight=*/1000);
periple::Solver solver(matrix, relaxed);
solver.nearest_neighbor();
```

`solver.cost()` returns the total distance plus penalties. When multiple windows exist, the violation is the minimum lateness across all windows.

`two_opt` minimizes that same sum, so a reversal is accepted when it trades distance for a smaller penalty or the reverse.

### Choosing penalty_weight

There is no universal value. The weight should be calibrated to the ratio between distance costs and violation amplitudes in your instance:
- Distances in [1, 100], violations in [0, 10]: a weight of 100 may suffice
- Distances in [1, 10000], violations in [0, 1]: a weight of 100000+ is needed

The default of 1000 is reasonable for typical TSPLIB-scale instances. Too low and the solver ignores windows; too high and it over-prioritizes feasibility at the expense of distance.

## Composition

Variants can be composed with `Composed` to combine multiple constraints:

```cpp
#include <periple/periple.hpp>            // Composed, ServiceTimes
#include <periple/variants/time_windows.hpp>

double durations[] = {0, 10, 0};
periple::time_windows::TimeWindow windows[] = {{0, 100}, {0, 100}, {0, 12}};

periple::service_times::ServiceTimes svc(durations);
periple::time_windows::Strict tw(windows);
auto variant = periple::Composed(svc, tw);   // ServiceTimes first, then Strict
periple::Solver solver(matrix, variant);
solver.nearest_neighbor();
```

**Order matters for `move_prepare`**: ServiceTimes adjusts the departure time, then Strict reads the adjusted arrival/departure to check feasibility. Place adjusters before verifiers.

**Dimension deduplication**: both `ServiceTimes` and `Strict` declare `dimension = RouteTiming`. `Composed` merges and deduplicates automatically -- the context contains a single `RouteTiming` instance shared by all variants.

Composed works with any combination:
- Two variants with the same dimension (shared state)
- A variant with a dimension + a variant without (mixed)
- Two variants without dimensions (no dimension state)

> **Important**: `Composed` stores pointers. Variant objects must outlive the `Composed` instance:
> ```cpp
> auto variant = Composed(svc, tw);  // OK: svc and tw are named
> Solver solver(mat, variant);
> ```

## Writing your own variant

A variant is a struct with optional `move_prepare` and `move_filter` methods. Implement only the callbacks you need.

### Minimal variant (no dimension)

```cpp
struct RejectCity {
    std::size_t forbidden;

    bool move_filter(const periple::AppendMove<std::size_t>& m) const {
        return m.city != forbidden;
    }
};

RejectCity reject{.forbidden = 3};
periple::Solver solver(matrix, reject);
solver.nearest_neighbor();
```

### Variant with cost adjustment

```cpp
struct Penalty {
    template <typename Ctx>
    void move_prepare(const periple::AppendMove<std::size_t>& m, Ctx& ctx) const {
        if (m.city == 1) ctx.cost_delta += 9999.0;
    }
};
```

### Variant with a dimension

If your variant needs to read or write time-based state (arrival, departure), declare `using dimension = RouteTiming`:

```cpp
struct MyTimeConstraint {
    using dimension = periple::RouteTiming;

    template <typename CityT, typename Ctx>
    void move_prepare(const periple::AppendMove<CityT>& m, Ctx& ctx) const {
        // Read arrival, adjust departure
        double arr = ctx.template dim<periple::RouteTiming>().arrival;
        ctx.template dim<periple::RouteTiming>().departure += processing_time(m.city);
    }

    template <typename CityT, typename Ctx>
    bool move_filter(const periple::AppendMove<CityT>& m, const Ctx& ctx) const {
        return ctx.template dim<periple::RouteTiming>().arrival <= deadline(m.city);
    }
};
```

### Variant supporting both constructive and exact algorithms

Provide overloads for both `AppendMove` and `DPMove`:

```cpp
struct MyVariant {
    using dimension = periple::RouteTiming;

    // Constructive algorithms
    template <typename CityT, typename Ctx>
    void move_prepare(const periple::AppendMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    bool move_filter(const periple::AppendMove<CityT>& m, const Ctx& ctx) const { /* ... */ }

    // Exact algorithms (Held-Karp)
    template <typename CityT, typename Ctx>
    void move_prepare(const periple::DPMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    bool move_filter(const periple::DPMove<CityT>& m, const Ctx& ctx) const { /* ... */ }
};
```

## Writing your own dimension

A dimension is a struct satisfying the `Dimension` concept:

```cpp
template <typename T>
concept Dimension = requires(T& t, std::size_t n) {
    t.resize(n);
    t.reset();
};
```

Additionally, a dimension should provide (detected by overload resolution, not the concept):

- `init(Move, Dist)` or `init(Move, Dist, Ctx)` -- initialize tentative state from the committed state of the previous position. The 3-param form receives the context, giving access to `tour()`, `position()`, and previously initialized dimensions.
- `commit(Move)` -- write tentative state to committed storage.
- `snapshot(Move) -> SnapshotType` and `restore(SnapshotType)` -- save and restore tentative state for constructive algorithms (evaluate/snapshot/restore pattern).

Each method is overloaded per move type (`AppendMove`, `DPMove`). See `RouteTiming` in `core/dimensions/route_timing.hpp` for a complete example.

### Example: custom dimension

```cpp
struct RouteLoad {
    // Tentative
    double load = 0;
    std::size_t pos_ = 0;

    // Committed (per-position)
    std::vector<double> loads;

    void resize(std::size_t n) { loads.resize(n); }
    void reset() { load = 0; std::fill(loads.begin(), loads.end(), 0.0); }

    // AppendMove: 3-param init (reads ctx for previous position)
    template <periple::DistanceSource Dist, typename CityT, typename Ctx>
    void init(const periple::AppendMove<CityT>& m, const Dist&, const Ctx& ctx) {
        pos_ = ctx.tour().size();
        load = (pos_ == 0) ? 0.0 : loads[pos_ - 1] + demand(m.city);
    }

    template <typename CityT>
    void commit(const periple::AppendMove<CityT>&) {
        loads[pos_] = load;
    }

    struct AppendSnapshot { double load; std::size_t pos; };
    template <typename CityT>
    auto snapshot(const periple::AppendMove<CityT>&) const -> AppendSnapshot {
        return {load, pos_};
    }
    void restore(const AppendSnapshot& s) { load = s.load; pos_ = s.pos; }
};
```

A variant can then use this dimension:

```cpp
struct CapacityConstraint {
    using dimension = RouteLoad;
    double max_load;

    template <typename CityT, typename Ctx>
    bool move_filter(const periple::AppendMove<CityT>&, const Ctx& ctx) const {
        return ctx.template dim<RouteLoad>().load <= max_load;
    }
};
```

Or combine dimensions:

```cpp
struct TimedCapacity {
    using dimension = periple::Dimensions<periple::RouteTiming, RouteLoad>;
    // ...
};
```
