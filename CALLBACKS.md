# Callback reference

Periple's callback architecture lets you customize algorithm behavior without modifying internal logic. Both variant callbacks and strategy callbacks are optional and zero-overhead when not used: `if constexpr` eliminates all checks at compile time for the default case.

## Two kinds of callbacks

- **Variant callbacks** describe the *problem* (constraints, adjusted costs, state tracking). Example: time windows, service times.
- **Strategy callbacks** define the *algorithm* inside a generic framework (which city to append next). Example: nearest-city selection.

Ready-to-use algorithms (`nearest_neighbor`, `held_karp`) have a built-in strategy and only accept variant callbacks. Generic frameworks (`greedy_construct`) take a strategy and optionally variant callbacks via the Solver's second template parameter.

Both use the same mechanism: structs with member functions detected via `if constexpr` + `requires`.

## Variant callbacks

A variant is a struct that provides any combination of two callbacks:

### `move_prepare`

Adjusts the evaluation context before scoring. Writes to `ctx.cost_delta` and/or dimension fields. Called once per candidate evaluation and once per committed placement.

Two forms, detected by `if constexpr`:

```cpp
// With context (accesses dimensions, cost_delta, tour, position, cost)
template <typename Ctx>
void move_prepare(const AppendMove<std::size_t>& m, Ctx& ctx) const {
    ctx.cost_delta += penalty;
    ctx.template dim<RouteTiming>().departure += service_time;
}

// Without context (stateless adjustment based on move alone)
void move_prepare(const AppendMove<std::size_t>& m) const;
```

### `move_filter`

Hard constraint. Return `false` to reject a move. Called after `move_prepare` so that filters can read the final dimension state. If absent, all moves are accepted.

Two forms:

```cpp
// With context (reads dimensions after prepare pipeline)
template <typename Ctx>
bool move_filter(const AppendMove<std::size_t>& m, const Ctx& ctx) const {
    return ctx.template dim<RouteTiming>().arrival <= deadline;
}

// Without context (decision based on move alone)
bool move_filter(const AppendMove<std::size_t>& m) const {
    return m.city != forbidden_city;
}
```

### Pipeline

For each candidate city, the Solver runs:

```
1. ctx.init(move, dist, tour, position, cost)   -- initialize dimensions
2. invoke_prepare(variant, move, ctx)            -- adjust cost_delta and dimensions
3. invoke_filter(variant, move, ctx)             -- check feasibility
4. score = dist(last, city) + ctx.cost_delta     -- final score
```

`invoke_prepare` and `invoke_filter` are dispatch helpers that detect callback presence and form (with/without context) via `if constexpr`.

## Move types

Each algorithm family defines its own move types. Variant callbacks provide overloads per move type. Both have a single template parameter (`CityT`).

| Move type | Algorithm family | Fields |
|---|---|---|
| `AppendMove<CityT>` | Constructive (NN, greedy) + closing edge + replay | `city` |
| `DPMove<CityT>` | Exact (Held-Karp) | `from`, `to`, `set` |

`DPMove::set` is a bitmask of visited cities (includes `from`, excludes `to`).

A variant supporting both constructive and exact algorithms provides overloads for both:

```cpp
struct MyVariant {
    template <typename CityT, typename Ctx>
    void move_prepare(const AppendMove<CityT>& m, Ctx& ctx) const { /* ... */ }

    template <typename CityT, typename Ctx>
    void move_prepare(const DPMove<CityT>& m, Ctx& ctx) const { /* ... */ }
};
```

## EvalContext

The evaluation context carries read-only solver state and mutable dimensions. It is created automatically by the Solver based on the variant's `dimension` typedef.

**Read-only** (set by `init`, not modifiable by callbacks):
- `ctx.tour()` -- current partial tour
- `ctx.position()` -- inverse index (city -> position)
- `ctx.cost()` -- current accumulated cost

**Mutable**:
- `ctx.cost_delta` -- additive adjustment to the move's score (`double`, always)
- `ctx.template dim<D>()` -- access dimension `D` (e.g., `RouteTiming`)

### Dimensions

A dimension is typed state that persists across the tour construction. It tracks tentative values (for evaluation) and committed values (for the tour built so far).

A variant declares its dimension(s) via a `dimension` typedef:

```cpp
struct MyVariant {
    using dimension = RouteTiming;                          // single dimension
    // or:
    using dimension = periple::Dimensions<RouteTiming, RouteEnergy>;  // multiple
};
```

If the variant has no `dimension` typedef, the Solver uses `EmptyContext` (no dimensions, just `cost_delta`).

`RouteTiming` is the built-in dimension provided by Periple. It tracks `arrival` and `departure` times per position (constructive) or per `(set, city)` state (DP).

### context_for deduction

The Solver deduces the context type automatically:

| Variant | Context |
|---|---|
| `NoCallbacks` | `EmptyContext<CityT, CostT>` |
| `using dimension = T` | `EvalContext<CityT, CostT, T>` |
| `using dimension = Dimensions<Ts...>` | `EvalContext<CityT, CostT, Ts...>` |

## Composition

Multiple variants can be combined via `Composed`:

```cpp
auto svc = service_times::ServiceTimes(durations);
auto tw = time_windows::Strict(windows);
auto variant = Composed(svc, tw);
Solver solver(matrix, variant);
```

`Composed<Vs...>`:
- Pipelines `move_prepare` through all variants in order (fold)
- AND-short-circuits `move_filter` through all variants in order
- Merges and deduplicates dimensions from all variants

Convention: place adjusters (ServiceTimes) before verifiers (Strict) so that filters see the adjusted state.

> **Important**: `Composed` stores pointers to the composed variants. The variant objects must outlive the `Composed` instance. Always use named variables, not temporaries:
> ```cpp
> // Correct:
> auto variant = Composed(svc, tw);
> Solver solver(mat, variant);
>
> // Wrong (dangling pointer):
> Solver solver(mat, Composed(svc, tw));  // Composed destroyed at end of expression
> ```

See [VARIANTS.md](VARIANTS.md) for pre-built variants and composition examples.

## Strategy callbacks

### `greedy_construct`

Builds a tour one city at a time. The strategy decides which city to append next.

**Required**: `select_next` returning `std::optional<city_type>`. Return `std::nullopt` to stop construction (the tour remains partial).

The strategy receives a const reference to the Solver, giving access to `tour()`, `position()`, `is_visited()`, `evaluate_append()`, and `distance()`:

```cpp
struct FarthestSelector {
    template <periple::DistanceSource Dist, typename Variant>
    auto select_next(const periple::Solver<Dist, Variant>& solver) const
        -> std::optional<typename periple::dist_traits<Dist>::city_type>
    {
        // Use solver.evaluate_append(city), solver.distance(a, b), etc.
    }
};

solver.greedy_construct(FarthestSelector{});
```

### `NearestSelector`

Built-in strategy for `greedy_construct`. Iterates unvisited cities, calls `solver.evaluate_append()` (which runs the full variant pipeline: init -> prepare -> filter -> score), and picks the minimum. This is what `nearest_neighbor()` uses internally.

```cpp
// These are equivalent:
solver.nearest_neighbor({.start_city = 0});
solver.greedy_construct(periple::NearestSelector{}, {.start_city = 0});
```

## Partial tours

`set_tour` accepts a prefix shorter than the full problem size. Algorithms can resume from a partial tour:

```cpp
std::vector<std::size_t> prefix = {0, 3, 7};
solver.set_tour(prefix);                                   // status = partial
solver.nearest_neighbor({.resume_at = solver.tour().size()});  // completes from the prefix
```

When resuming, `rebuild_and_cost` replays the prefix through the full pipeline (init + prepare + commit for each city), reconstructing dimension state and cost. No manual state management is needed.

## Local search and replay

Local search does not append cities, it rewrites part of an existing tour. Instead of a second callback interface, Periple replays the changed suffix through the constructive pipeline: every position from the first changed one is evaluated as an `AppendMove`, so a variant written for `nearest_neighbor` works with `two_opt` unchanged.

During a replay `ctx.tour()` and `ctx.position()` are empty spans, since the candidate tour exists nowhere yet, and `ctx.cost()` is the running cost of the candidate prefix.

Any variant gets a `CumulativeCost` dimension automatically. It holds the committed cost of every prefix, which is what makes a partial replay possible: scoring a change at position `i` starts from `prefix_cost(i)` rather than replaying the whole tour.

| Method | Description |
|---|---|
| `prefix_cost(pos)` | Committed cost of `tour[0..pos]`, penalties included. O(1) |
| `evaluate_reversal(i, j)` | Cost of the tour with `tour[i+1..j]` reversed, `nullopt` if a filter rejects it. Leaves the evaluation staged. O(n - i) |
| `accept_reversal(i, j, cost)` | Applies a scored reversal: commits the staged dimensions, then reverses. O(j - i) |
| `evaluate_replay(city_at, from, prefix)` | Same for an arbitrary rewrite: `city_at(pos)` returns the candidate city at each position |
| `accept_replay(city_at, from, cost)` | Applies a scored rewrite. `city_at` must not read `solver.tour()`, which it overwrites as it goes |
| `discard_staging()` | Drops a staged evaluation that was not accepted |
| `path_cost(from, to)` | Raw distance along the tour between two positions, either direction. O(1), without a variant |

A complete operator, first improvement over the neighbor lists:

```cpp
// Reverses the first improving segment found. Returns false at a local optimum.
template <typename Solver>
bool improve_once(Solver& solver) {
    const auto tour = solver.tour();
    for (std::size_t i = 0; i + 2 < tour.size(); ++i) {
        for (auto c : solver.neighbors(tour[i], 5)) {
            const auto j = static_cast<std::size_t>(solver.position()[c]);
            if (j < i + 2) continue;
            const auto scored = solver.evaluate_reversal(i, j);
            if (scored && *scored < solver.cost()) {
                solver.accept_reversal(i, j, *scored);
                return true;
            }
        }
    }
    solver.discard_staging();  // nothing accepted: drop the last evaluation
    return false;
}
```

`evaluate_reversal` and `accept_reversal` are the pair `two_opt` uses in replay mode. Without a variant there is nothing to stage: compute the delta from `path_cost` and pass the resulting cost to `accept_reversal`.

## Variant callbacks and symmetry

Symmetry is a property of the **full problem** (distance matrix + variant callbacks), not of the distance matrix alone. Variant callbacks can break symmetry even on a symmetric matrix (e.g., direction-dependent time windows).

Use `set_symmetric` to declare the effective symmetry. In debug builds, `set_symmetric(true)` asserts that the distance matrix is symmetric. Use `set_symmetric(true, periple::unchecked)` to skip this check.

```cpp
solver.set_symmetric(true);                       // symmetric problem
solver.set_symmetric(false);                      // asymmetric (e.g., TSPTW)
solver.set_symmetric(true, periple::unchecked);   // trust me, skip check
```

## Applicability

| Family | `move_prepare` | `move_filter` |
|---|---|---|
| Constructive (NN, greedy) | yes | yes |
| Exact (Held-Karp) | yes | yes |
| Local search (2-opt) | yes | yes (replayed as `AppendMove`) |

## Variants

Pre-built variant callbacks for common TSP variants are documented in [VARIANTS.md](VARIANTS.md).
