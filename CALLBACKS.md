# Callbacks reference

Periple's callback architecture lets you customize algorithm behavior without modifying internal logic. Callbacks are optional and zero-overhead when not used: `if constexpr` eliminates all callback checks at compile time for the default case.

## Overview

There are two levels of customization:

- **TourCost** (Solver-level): controls how the total cost of a complete tour is computed. Set as the second template parameter of `Solver`.
- **Per-algorithm callbacks** (move-level): control filtering, evaluation, scoring, and notification for individual moves. Passed as an argument to each algorithm call.

Both use the same mechanism: structs with optional member functions detected via `if constexpr` + `requires`.

## The 5 callbacks

### 1. `tour_cost` (Solver-level)

Computes the cost of a complete tour. Called after construction, after `set_tour()`, or after an algorithm completes. If absent (`DefaultTourCost`), the default is the sum of distances along the tour.

```cpp
struct MyTourCost {
    template <periple::DistanceSource Dist>
    auto operator()(const Dist& dist,
                    std::span<const typename periple::dist_traits<Dist>::city_type> tour) const
        -> typename periple::dist_traits<Dist>::cost_type
    {
        // Custom cost computation
    }
};

periple::Solver solver(matrix, MyTourCost{});
```

### 2. `move_filter` (per-algorithm)

Determines if a move is feasible (hard constraint). Called before `move_eval`/`move_score`. If absent, all moves are accepted. Return `false` to reject a move.

```cpp
bool move_filter(std::span<const city_type> tour, const AppendMove<city_type>& m) const;
```

### 3. `move_eval` (per-algorithm)

Computes the real cost of a move. Used to update `cost_` and track the best solution. Replaces distance when it is insufficient (e.g., distance + time window penalties). If absent, the default is the distance delta.

```cpp
auto move_eval(std::span<const city_type> tour, const AppendMove<city_type>& m) const -> cost_type;
```

### 4. `move_score` (per-algorithm)

Biased scoring for search guidance (GLS, diversification, learning). Used for accept/reject decisions. Does NOT affect `cost_`/best-solution tracking. If absent, `move_eval` is used (or distance if `move_eval` is also absent).

```cpp
auto move_score(std::span<const city_type> tour, const AppendMove<city_type>& m) const -> cost_type;
```

### 5. `on_commit` (per-algorithm)

Called after a move has been accepted and applied. Use it to update user caches (prefix sums, segment trees, arrival times, etc.).

```cpp
void on_commit(std::span<const city_type> tour, const AppendMove<city_type>& m) const;
```

### Priority rules

The metric used for decisions follows a universal priority: `move_score` > `move_eval` > `dist`. The metric used for cost tracking follows: `move_eval` > `dist`.

| `move_score` | `move_eval` | Decisions | Cost tracking |
|---|---|---|---|
| absent | absent | distance | distance |
| absent | present | `move_eval` | `move_eval` |
| present | absent | `move_score` | distance |
| present | present | `move_score` | `move_eval` |

## Move types

Each algorithm defines its own move types. Callbacks are overloaded per move type.

### Current move types

| Move type | Algorithm family | Fields |
|---|---|---|
| `AppendMove<CityT>` | Constructive (NN, greedy) | `CityT city` |

### Future move types (not yet implemented)

| Move type | Algorithm family | Fields |
|---|---|---|
| `TwoOptMove` | Local search | `std::size_t i, j` |
| `OrOptMove` | Local search | `std::size_t pos, len, target` |
| `DoubleBridgeMove` | Perturbation | TBD |

## Selectors (constructive algorithms)

A selector defines the **strategy** (minimize, maximize, random). The **metric** is provided by callbacks via the priority chain. Two orthogonal concerns.

### Built-in selectors

**`NearestSelector`**: minimizes the metric. This is what `nearest_neighbor()` uses internally.

```cpp
#include <periple/algorithms/nearest_neighbor.hpp>
```

### Custom selectors

A selector must implement an `evaluate` method:

```cpp
struct MySelector {
    template <periple::DistanceSource Dist, typename Callbacks>
    auto evaluate(const Dist& dist,
                  std::span<const typename periple::dist_traits<Dist>::city_type> tour,
                  typename periple::dist_traits<Dist>::city_type candidate,
                  const Callbacks& cb) const
        -> typename periple::dist_traits<Dist>::cost_type
    {
        // Return a score; the algorithm selects the candidate with the lowest score.
    }
};

solver.greedy_construct(MySelector{});
```

### greedy_construct vs nearest_neighbor

`nearest_neighbor()` is a facade for `greedy_construct(NearestSelector{})`. Use `greedy_construct` when you need a custom selector:

```cpp
// These are equivalent:
solver.nearest_neighbor({.start_city = 0}, my_callbacks);
solver.greedy_construct(NearestSelector{}, my_callbacks, {.start_city = 0});
```

## Partial tours

`set_tour` accepts a prefix shorter than the full problem size:

```cpp
std::vector<std::size_t> prefix = {0, 3, 7};
solver.set_tour(prefix);                    // status = partial
solver.nearest_neighbor();                  // completes from the prefix
```

| Operation | Status | Cost |
|---|---|---|
| `set_tour(full)` | `feasible` | `tour_cost(tour)` |
| `set_tour(prefix)` | `partial` | open-path distance |
| Algorithm completes | `feasible`/`optimal` | `tour_cost(full)` |

When continuing from a partial tour, `on_commit` is only called for newly placed cities. If your callbacks maintain state (like arrival times), ensure they are consistent with the existing prefix before calling the algorithm.

## Variants

Pre-built callback structs for common TSP variants (TSPTW, etc.) are documented in [VARIANTS.md](VARIANTS.md).

## Applicability by algorithm family

| Family | `tour_cost` | `move_filter` | `move_eval` | `move_score` | `on_commit` | Move types |
|---|---|---|---|---|---|---|
| Constructive (NN) | yes | yes | yes | yes | yes | `AppendMove` |
| Local search (future) | yes | yes | yes | yes | yes | `TwoOptMove`, `OrOptMove` |
| Exact (HK) | yes | - | - | - | - | - |

## Testing callbacks

### LoggingCallbacks

A test utility that records every callback invocation:

```cpp
#include "logging_callbacks.hpp"

periple::LoggingCallbacks cb;
solver.nearest_neighbor({}, cb);
// cb.log contains "move_filter" and "on_commit" entries in order
```

### Test categories

- **Regression**: `DefaultCallbacks` produce the same results as before callbacks existed.
- **Protocol**: `LoggingCallbacks` verify invocation order and count.
- **Functional**: Variant callbacks (e.g., `tsptw::Strict`) produce correct constrained behavior.
- **Unit**: Simple custom callbacks verify specific behaviors (rejection, custom cost, custom selector, partial tour completion).
