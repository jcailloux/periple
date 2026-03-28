# Callback reference

Periple's callback architecture lets you customize algorithm behavior without modifying internal logic. Both variant callbacks and strategy callbacks are optional and zero-overhead when not used: `if constexpr` eliminates all checks at compile time for the default case.

## Two kinds of callbacks

- **Variant callbacks** describe the *problem* (constraints, adjusted costs, state tracking). Example: TSPTW time windows.
- **Strategy callbacks** define the *algorithm* inside a generic framework (which city to append, how to bound, etc.). Example: nearest-city selection.

Ready-to-use algorithms (`nearest_neighbor`, future `two_opt`) have a built-in strategy and only accept variant callbacks. Generic frameworks (`greedy_construct`, future `branch_and_bound`) take a strategy and optionally variant callbacks, which the framework forwards to the strategy.

Both use the same mechanism: structs with member functions detected via `if constexpr` + `requires`.

## Variant callbacks

### `tour_cost` (Solver-level)

Computes the cost of a complete tour. Set as the second template parameter of `Solver`. If absent (`DefaultTourCost`), the default is the sum of edge distances.

```cpp
struct MyTourCost {
    template <periple::DistanceSource Dist>
    auto operator()(const Dist& dist,
                    std::span<const typename periple::dist_traits<Dist>::city_type> tour) const
        -> typename periple::dist_traits<Dist>::cost_type;
};

periple::Solver solver(matrix, MyTourCost{});
```

### `move_filter`

Hard constraint. Return `false` to reject a move. If absent, all moves are accepted.

```cpp
bool move_filter(std::span<const city_type> tour, const AppendMove<city_type>& m) const;
```

### `move_eval`

Adjusted cost of a move. Replaces distance when it is insufficient (e.g., distance + penalty). If absent, the raw distance is used.

```cpp
auto move_eval(std::span<const city_type> tour, const AppendMove<city_type>& m) const -> cost_type;
```

### `move_score`

Biased scoring for search guidance (GLS, diversification). Used for selection decisions but does NOT affect cost tracking. If absent, `move_eval` is used (or distance if `move_eval` is also absent).

```cpp
auto move_score(std::span<const city_type> tour, const AppendMove<city_type>& m) const -> cost_type;
```

### `on_commit`

Called after a move is applied. Use it to update caches (arrival times, load sums, etc.).

```cpp
void on_commit(std::span<const city_type> tour, const AppendMove<city_type>& m) const;
```

### `on_improve`

Called when a DP transition yields a new best for a state. Use it to maintain auxiliary state (arrival times, accumulated fatigue, etc.) alongside the DP table.

```cpp
void on_improve(const DPMove<city_type, cost_type>& m) const;
```

### Priority rules

| `move_score` | `move_eval` | Selection | Cost tracking |
|---|---|---|---|
| absent | absent | distance | distance |
| absent | present | `move_eval` | `move_eval` |
| present | absent | `move_score` | distance |
| present | present | `move_score` | `move_eval` |

## Move types

Each algorithm family defines its own move types. Variant callbacks are overloaded per move type.

| Move type | Algorithm family | Fields |
|---|---|---|
| `AppendMove<CityT>` | Constructive | `city` |
| `DPMove<CityT, CostT>` | Exact (HK) | `from`, `to`, `cost`, `distance`, `set` |

`DPMove` fields: `from` and `to` are the cities, `cost` is the cumulative DP cost at `from` (after move_eval adjustments), `distance` is the raw `dist(from, to)`, and `set` is the bitmask of visited cities (includes `from`, excludes `to`). The variant callback can maintain auxiliary state indexed by `(set, city)`.

Future: `TwoOptMove`, `OrOptMove`, `DoubleBridgeMove`.

## Strategy callbacks

### `greedy_construct`

Builds a tour one city at a time. The strategy decides which city to append next.

**Required**: `select_next` returning `std::optional<city_type>`. Return `std::nullopt` to stop construction (the tour remains partial).

**Optional**: `on_placed`, called after each city is placed in the tour buffer.

Both methods exist in two forms: a 3-argument form that ignores variant callbacks, and a 4-argument form that receives them. If variant callbacks are passed to `greedy_construct`, the framework forwards them to whichever form the strategy provides:

```cpp
// 3-arg: standalone strategy, no variant callbacks needed
struct FarthestSelector {
    template <periple::DistanceSource Dist>
    auto select_next(const Dist& dist,
                     std::span<const city_type> partial_tour,
                     std::span<const uint8_t> visited) const
        -> std::optional<city_type>;
};

solver.greedy_construct(FarthestSelector{});
```

```cpp
// 4-arg: strategy that uses variant callbacks
struct ConstrainedSelector {
    template <periple::DistanceSource Dist, typename Variant>
    auto select_next(const Dist& dist,
                     std::span<const city_type> partial_tour,
                     std::span<const uint8_t> visited,
                     const Variant& variant) const
        -> std::optional<city_type>
    {
        // variant.move_filter(), variant.move_eval(), etc.
    }
};

solver.greedy_construct(ConstrainedSelector{}, tw_strict);
```

### `NearestSelector`

Built-in strategy for `greedy_construct`. Iterates unvisited cities, applies `move_filter` (if present), scores via `move_score > move_eval > dist`, picks the minimum, and delegates `on_commit` through `on_placed`. This is what `nearest_neighbor()` uses internally.

Without variant callbacks, it scores by raw distance.

```cpp
// These are equivalent:
solver.nearest_neighbor(tw, {.start_city = 0});
solver.greedy_construct(periple::NearestSelector{}, tw, {.start_city = 0});
```

## Partial tours

`set_tour` accepts a prefix shorter than the full problem size:

```cpp
std::vector<std::size_t> prefix = {0, 3, 7};
solver.set_tour(prefix);                    // status = partial
solver.nearest_neighbor();                  // completes from the prefix
```

When continuing from a partial tour, `on_commit` is only called for newly placed cities. If your variant callbacks maintain state (like arrival times), ensure they are consistent with the existing prefix before calling the algorithm.

## Variant callbacks and symmetry

Symmetry is a property of the **full problem** (distance matrix + variant callbacks), not of the distance matrix alone. Variant callbacks can break symmetry even on a symmetric matrix (e.g., direction-dependent costs), or restore it on an asymmetric one.

Use `set_symmetric` to declare the effective symmetry. In debug builds, `set_symmetric(true)` asserts that the distance matrix is symmetric. Use `set_symmetric(true, periple::unchecked)` to skip this check.

```cpp
solver.set_symmetric(true);                       // symmetric problem
solver.set_symmetric(false);                      // asymmetric (e.g., TSPTW)
solver.set_symmetric(true, periple::unchecked);   // trust me, skip check
```

## Applicability

| Family | `tour_cost` | `move_filter` | `move_eval` | `move_score` | `on_commit` | `on_improve` |
|---|---|---|---|---|---|---|
| Constructive (NN) | yes | yes | yes | yes | yes | - |
| Exact (HK) | yes | yes | yes | - | - | yes |
| Local search (future) | yes | yes | yes | yes | yes | - |

## Variants

Pre-built variant callbacks for common TSP variants (TSPTW, etc.) are documented in [VARIANTS.md](VARIANTS.md).
