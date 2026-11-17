#pragma once

// Held-Karp algorithm (exact, dynamic programming)
//
// Bellman (1962), "Dynamic Programming Treatment of the Travelling Salesman Problem"
//   -- DP over sets of cities from city 0; order constraints restrict its minimization
// Held, Karp (1962), "A Dynamic Programming Approach to Sequencing Problems"
//   -- C(S, l), from the first city through S to l, closed into a tour, eq. (5) to (8);
//      precedences restrict the class of sets, sections 1.3 and 1.4
// Dumas, Desrosiers, Gelinas, Solomon (1995), "An Optimal Algorithm for the Traveling
//   Salesman Problem with Time Windows"
//   -- forward DP whose step s generates the paths of length s; per (set, city), only
//      the states that no other dominates, section 1; a state is dropped once an
//      unvisited city or the end of the tour can no longer be reached by its deadline,
//      even by the shortest path without windows, test 1

#include <periple/core/solver.hpp>
#include <periple/core/latest_departures.hpp>
#include <periple/core/moves/dp_move.hpp>

#include <algorithm>
#include <array>
#include <bit>
#include <cassert>
#include <compare>
#include <concepts>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>
#include <vector>

namespace periple {

// held_karp engines. AllSets: a table over every visited set, up to 25 cities.
// ReachableSets: only the sets the variant's DPMove filter lets the DP reach,
// up to 64 cities; far faster when the filter prunes, slower when it prunes little.
// Both keep the cheapest state per (set, city). Passed where that may miss the
// optimum, they report feasible instead of optimal, and partial (the start city
// alone) instead of infeasible, once they drop a state that no other dominates.
// ReachableLabels: the sets of ReachableSets, with every state per (set, city)
// that no other dominates.
struct AllSets {};
struct ReachableSets {};
struct ReachableLabels {};

namespace detail {

// A variant prunes the DP only through a DPMove filter; a Composed through any
// of its components'.
template <typename V, typename Ctx, typename CityT>
struct filters_dp : std::bool_constant<
	requires(const V& v, const DPMove<CityT>& m, const Ctx& ctx) { { v.move_filter(m, ctx) } -> std::convertible_to<bool>; } ||
	requires(const V& v, const DPMove<CityT>& m) { { v.move_filter(m) } -> std::convertible_to<bool>; }> {};

template <typename... Vs, typename Ctx, typename CityT>
struct filters_dp<Composed<Vs...>, Ctx, CityT>
	: std::bool_constant<(filters_dp<Vs, Ctx, CityT>::value || ...)> {};

// A variant needs every non-dominated state when one of its dimensions keeps DP
// state and it does not declare that the cheapest suffices; a Composed when any
// of its components does.
template <typename Dims> struct stores_any_dp_state;
template <typename... Ds>
struct stores_any_dp_state<Dimensions<Ds...>> : std::bool_constant<(stores_dp_state<Ds> || ...)> {};

template <typename V>
struct needs_labels : std::bool_constant<
	stores_any_dp_state<typename declared_dims<V>::type>::value && !cheapest_state_suffices<V>::value> {};

template <typename... Vs>
struct needs_labels<Composed<Vs...>> : std::bool_constant<(needs_labels<Vs>::value || ...)> {};

// The engine of held_karp(): ReachableLabels where one state per (set, city)
// may miss the optimum, else ReachableSets where a filter can prune, AllSets
// elsewhere, where ReachableSets would reach every set at a higher cost.
template <typename Variant, typename Ctx, typename CityT>
using default_hk_engine = std::conditional_t<needs_labels<Variant>::value, ReachableLabels,
	std::conditional_t<filters_dp<Variant, Ctx, CityT>::value, ReachableSets, AllSets>>;

// Under one state per (set, city), whether a candidate that meets the stored
// state loses one that no other dominates: the candidate dropped while the stored
// state does not dominate it, or the stored state replaced by a candidate that
// does not dominate it.
template <typename Ctx, typename CostT>
bool loses_state(const Ctx& ctx, CostT candidate, CostT stored, std::size_t state) {
	if (stored == std::numeric_limits<CostT>::max()) return false;
	const auto order = ctx.compare_dp(state);
	return candidate < stored ? !(order <= 0) : !(order >= 0);
}

} // namespace detail

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::held_karp(HeldKarpParams params) -> Solver& {
	return held_karp(detail::default_hk_engine<Variant, context_type, city_type>{}, params);
}

template <DistanceSource Dist, typename Variant>
template <typename Engine>
auto Solver<Dist, Variant>::held_karp(Engine, HeldKarpParams params) -> Solver& {
	static_assert(std::is_same_v<Engine, AllSets> || std::is_same_v<Engine, ReachableSets>
		|| std::is_same_v<Engine, ReachableLabels>,
		"held_karp: the engine is periple::AllSets, periple::ReachableSets or periple::ReachableLabels");
	static_assert(detail::covers_move<Variant, DPMove<city_type>, context_type, city_type>::value,
		"held_karp: the variant, or one of its components, has no DPMove callback, so held_karp would ignore it");
	static_assert(context_type::template dims_handle<DPMove<city_type>, Dist>,
		"held_karp: a dimension has no DPMove init or commit, so its state would not follow the DP");

	if (try_trivial()) {
		status_ = SolutionStatus::optimal;
		return *this;
	}
	return detail::with_deadline(params.time_limit, [&](auto& deadline) -> Solver& {
		if constexpr (std::is_same_v<Engine, AllSets>) return held_karp_all_sets(deadline);
		else if constexpr (std::is_same_v<Engine, ReachableSets>) return held_karp_reachable(deadline);
		else return held_karp_labels(deadline);
	});
}

// Stores at root the start city's state as append() leaves it, so a tour's DP
// cost and its replay by rebuild_and_cost start from the same state.
template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::start_dp(std::size_t root) {
	AppendMove<city_type> start{city_type{0}, city_type{}, 0};
	ctx_.init(start, *dist_, {tour_.data(), 0}, {position_.data(), dist_->size()}, cost_type{});
	invoke_prepare(variant(), start, ctx_);
	ctx_.begin_dp(root);
}

// Under due times, the latest departures toward them, for dp_dead; false if
// stop fires first.
template <DistanceSource Dist, typename Variant>
template <typename Stop>
bool Solver<Dist, Variant>::start_dp_deadlines(Stop& stop) {
	if (!hk_due_) hk_due_.emplace();
	std::vector<double> due(n_);
	detail::dp_deadlines<Variant>::due_times(variant(), due);
	const auto distance = [this](std::size_t a, std::size_t b) {
		return (*dist_)(static_cast<city_type>(a), static_cast<city_type>(b));
	};
	if (!detail::build_latest_departures(*hk_due_, distance, due, stop)) return false;
	detail::build_least_slacks(*hk_due_, n_);
	return true;
}

// Whether the state just prepared, at j with the cities of next, can no longer
// reach an unvisited city or city 0 in time; false without due times. Such a
// state completes no tour: the engines test it only where they would store it
// or count it lost, and do neither. Inline: without it, GCC keeps it out of
// line, one call per state tested.
template <DistanceSource Dist, typename Variant>
inline bool Solver<Dist, Variant>::dp_dead(std::size_t j, std::size_t next) const {
	if constexpr (detail::dp_deadlines<Variant>::value) {
		const double departure = detail::dp_deadlines<Variant>::departure(ctx_);
		return detail::may_miss_a_due_time(*hk_due_, j, departure)
			&& detail::misses_a_due_time(*hk_due_, n_, j, departure, 0,
				[next](std::size_t c) { return ((next >> c) & 1) == 0; });
	} else {
		return false;
	}
}

template <DistanceSource Dist, typename Variant>
template <typename D>
auto Solver<Dist, Variant>::held_karp_all_sets(D& deadline) -> Solver& {
	// The table gives no tour until it is complete: back to the start city.
	auto give_up = [this]() -> Solver& {
		clear();
		append(city_type{0});
		return *this;
	};
	detail::Poller stop_fill(deadline);  // table allocation and fill
	if (stop_fill()) return give_up();

	constexpr auto INF = std::numeric_limits<cost_type>::max();

	n_ = dist_->size();
	assert(n_ <= 25 && "held_karp: AllSets holds 2^(n-1) sets, up to 25 cities; ReachableSets takes up to 64");
	ensure_capacity(n_);
	if constexpr (detail::dp_deadlines<Variant>::value)
		if (!start_dp_deadlines(stop_fill)) return give_up();

	if (!hk_cache_) hk_cache_.emplace();

	const std::size_t num_sets = std::size_t{1} << n_;
	const std::size_t table_sz = (num_sets >> 1) * n_;  // odd sets only
	hk_cache_->reserve(table_sz);

	auto dp     = std::span<cost_type>(hk_cache_->dp.get(), table_sz);
	auto parent = std::span<std::uint8_t>(hk_cache_->parent.get(), table_sz);

	// Every set holds city 0: S >> 1 drops that bit.
	auto idx = [this](std::size_t S, std::size_t i) -> std::size_t {
		return (S >> 1) * n_ + i;
	};

	// Only sets containing city 0 (odd S) are stored.
	for (std::size_t S = 1; S < num_sets; S += 2) {
		if (stop_fill()) return give_up();
		std::fill_n(dp.begin() + static_cast<std::ptrdiff_t>(idx(S, 0)), n_, INF);
	}

	// Base: start at city 0
	dp[idx(1, 0)] = cost_type{};
	ctx_.resize_dp(table_sz);
	start_dp(idx(1, 0));

	const auto& v = variant();

	// Under a variant that needs labels: a state that no other dominates was dropped.
	bool lost = false;

	// Forward DP, polled apart: its units cost far more than the fill's.
	detail::Poller stop_forward(deadline);
	const std::size_t complement_mask = num_sets - 1;
	for (std::size_t S = 1; S < num_sets; S += 2) { // S += 2: city 0 always in set
		for (auto si = S; si; si &= si - 1) {
			auto i = static_cast<std::size_t>(std::countr_zero(si));
			if (dp[idx(S, i)] == INF) continue;
			if (stop_forward()) return give_up();

			auto ci = static_cast<city_type>(i);

			for (auto sj = ~S & complement_mask; sj; sj &= sj - 1) {
				auto j = static_cast<std::size_t>(std::countr_zero(sj));
				auto cj = static_cast<city_type>(j);
				auto raw_dist = (*dist_)(ci, cj);

				DPMove<city_type> move{ci, cj, S, idx(S, i)};
				ctx_.init(move, *dist_, dp[idx(S, i)]);
				invoke_prepare(v, move, ctx_);

				if (!invoke_filter(v, move, ctx_)) continue;

				cost_type edge_cost = static_cast<cost_type>(static_cast<double>(raw_dist) + ctx_.cost_delta);

				std::size_t S_next = S | (std::size_t{1} << j);
				cost_type new_cost = dp[idx(S, i)] + edge_cost;
				if constexpr (detail::needs_labels<Variant>::value)
					lost = lost || (detail::loses_state(ctx_, new_cost, dp[idx(S_next, j)], idx(S_next, j))
						&& !dp_dead(j, S_next));
				if (new_cost < dp[idx(S_next, j)]) {
					if (dp_dead(j, S_next)) continue;
					dp[idx(S_next, j)]     = new_cost;
					parent[idx(S_next, j)] = static_cast<std::uint8_t>(i);
					move.to_state = idx(S_next, j);
					ctx_.commit(move);
				}
			}
		}
	}

	// Find optimal last city (close the tour back to 0)
	const std::size_t full = num_sets - 1;
	cost_type best_cost = INF;
	city_type best_last{};

	for (std::size_t i = 1; i < n_; ++i) {
		if (dp[idx(full, i)] == INF) continue;

		auto ci = static_cast<city_type>(i);
		auto raw_dist = (*dist_)(ci, city_type{0});

		DPMove<city_type> move{ci, city_type{0}, full, idx(full, i)};
		ctx_.init(move, *dist_, dp[idx(full, i)]);
		invoke_prepare(v, move, ctx_);

		if (!invoke_filter(v, move, ctx_)) continue;

		cost_type edge_cost = static_cast<cost_type>(static_cast<double>(raw_dist) + ctx_.cost_delta);

		cost_type c = dp[idx(full, i)] + edge_cost;
		if (c < best_cost) {
			best_cost = c;
			best_last = ci;
		}
	}

	if (best_cost == INF) {
		// A dropped state may have completed the tour.
		if (lost) return give_up();
		n_ = 0;
		cost_ = {};
		++tour_version_;
		status_ = SolutionStatus::infeasible;
		return *this;
	}

	// Backtrack
	tour_[n_ - 1] = best_last;
	std::size_t S = full;
	for (std::size_t pos = n_ - 1; pos > 0; --pos) {
		auto cur = static_cast<std::size_t>(tour_[pos]);
		tour_[pos - 1] = static_cast<city_type>(parent[idx(S, cur)]);
		S ^= (std::size_t{1} << cur);
	}
	std::fill_n(visited_.data(), n_, uint8_t{1});
	++tour_version_;

	// A dropped state may have completed it at a lower cost.
	cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = lost ? SolutionStatus::feasible : SolutionStatus::optimal;
	return *this;
}

template <DistanceSource Dist, typename Variant>
template <typename D>
auto Solver<Dist, Variant>::held_karp_reachable(D& deadline) -> Solver& {
	// The table gives no tour until it is complete: back to the start city.
	auto give_up = [this]() -> Solver& {
		clear();
		append(city_type{0});
		return *this;
	};
	detail::Poller stop(deadline);
	if (stop()) return give_up();

	constexpr auto INF = std::numeric_limits<cost_type>::max();
	constexpr auto unresolved = std::numeric_limits<std::uint32_t>::max();
	constexpr auto mask_bits = static_cast<std::size_t>(std::numeric_limits<std::size_t>::digits);

	n_ = dist_->size();
	assert(n_ <= mask_bits && "held_karp: instance too large (visited sets are std::size_t bitmasks)");
	ensure_capacity(n_);
	if constexpr (detail::dp_deadlines<Variant>::value)
		if (!start_dp_deadlines(stop)) return give_up();

	if (!hk_states_cache_) hk_states_cache_.emplace();
	auto& c = *hk_states_cache_;
	c.masks.clear();
	c.index.clear();
	if (c.target_slot.size() < n_) c.target_slot.resize(n_);

	// Appends an unreached slot for mask, growing every per-state buffer,
	// dimensions included, geometrically.
	std::size_t state_cap = 0;
	auto new_slot = [&](std::size_t mask) -> std::uint32_t {
		const std::size_t slot = c.masks.size();
		const std::size_t end = (slot + 1) * n_;
		if (end > state_cap) {
			state_cap = std::max(end, 2 * state_cap);
			assert(state_cap <= unresolved && "held_karp: too many states for 32-bit indices");
			if (c.cost.size() < state_cap) {
				c.cost.resize(state_cap);
				c.pred.resize(state_cap);
			}
			ctx_.resize_dp(state_cap);
		}
		std::fill_n(c.cost.begin() + static_cast<std::ptrdiff_t>(slot * n_), n_, INF);
		c.masks.push_back(mask);
		return static_cast<std::uint32_t>(slot);
	};

	// Base: slot 0 is {0}, start at city 0
	new_slot(1);
	c.cost[0] = cost_type{};
	start_dp(0);

	const auto& v = variant();
	const std::size_t full = ~std::size_t{0} >> (mask_bits - n_);

	// Under a variant that needs labels: a state that no other dominates was dropped.
	bool lost = false;

	// Forward DP, one layer (set size) after another: the slots a layer creates
	// are appended after it, so slot order is layer order.
	std::size_t layer_end = 1;
	for (std::size_t s = 0; s < c.masks.size(); ++s) {
		if (s == layer_end) {
			c.index.clear();
			layer_end = c.masks.size();
		}

		const std::size_t S = c.masks[s];
		std::fill_n(c.target_slot.begin(), n_, unresolved);

		for (auto si = S; si; si &= si - 1) {
			const auto i = static_cast<std::size_t>(std::countr_zero(si));
			const std::size_t from_state = s * n_ + i;
			if (c.cost[from_state] == INF) continue;
			if (stop()) return give_up();

			const auto ci = static_cast<city_type>(i);

			for (auto sj = ~S & full; sj; sj &= sj - 1) {
				const auto j = static_cast<std::size_t>(std::countr_zero(sj));
				const auto cj = static_cast<city_type>(j);

				DPMove<city_type> move{ci, cj, S, from_state};
				ctx_.init(move, *dist_, c.cost[from_state]);
				invoke_prepare(v, move, ctx_);

				if (!invoke_filter(v, move, ctx_)) continue;

				const auto edge_cost = static_cast<cost_type>(static_cast<double>((*dist_)(ci, cj)) + ctx_.cost_delta);
				const cost_type new_cost = c.cost[from_state] + edge_cost;

				// Slot of S | {j}, created by the first accepted move into it that
				// leaves a live state.
				auto& t = c.target_slot[j];
				if (t == unresolved) {
					const std::size_t next = S | (std::size_t{1} << j);
					if (dp_dead(j, next)) continue;
					t = c.index.find_or_insert(next, [&] { return new_slot(next); });
				}

				const std::size_t to_state = std::size_t{t} * n_ + j;
				if constexpr (detail::needs_labels<Variant>::value)
					lost = lost || (detail::loses_state(ctx_, new_cost, c.cost[to_state], to_state)
						&& !dp_dead(j, S | (std::size_t{1} << j)));
				if (new_cost < c.cost[to_state]) {
					if (dp_dead(j, S | (std::size_t{1} << j))) continue;
					c.cost[to_state] = new_cost;
					c.pred[to_state] = static_cast<std::uint32_t>(from_state);
					move.to_state = to_state;
					ctx_.commit(move);
				}
			}
		}
	}

	// Close the tour back to 0 from the full set, the last slot when reached.
	const std::size_t last = c.masks.size() - 1;
	cost_type best_cost = INF;
	std::size_t best_state = 0;

	if (c.masks[last] == full) {
		for (std::size_t i = 1; i < n_; ++i) {
			const std::size_t from_state = last * n_ + i;
			if (c.cost[from_state] == INF) continue;

			const auto ci = static_cast<city_type>(i);

			DPMove<city_type> move{ci, city_type{0}, full, from_state};
			ctx_.init(move, *dist_, c.cost[from_state]);
			invoke_prepare(v, move, ctx_);

			if (!invoke_filter(v, move, ctx_)) continue;

			const auto edge_cost = static_cast<cost_type>(static_cast<double>((*dist_)(ci, city_type{0})) + ctx_.cost_delta);
			const cost_type total = c.cost[from_state] + edge_cost;
			if (total < best_cost) {
				best_cost = total;
				best_state = from_state;
			}
		}
	}

	if (best_cost == INF) {
		// A dropped state may have completed the tour.
		if (lost) return give_up();
		n_ = 0;
		cost_ = {};
		++tour_version_;
		status_ = SolutionStatus::infeasible;
		return *this;
	}

	// Backtrack through predecessor states; a state's city is state % n.
	std::size_t state = best_state;
	for (std::size_t pos = n_ - 1; pos > 0; --pos) {
		tour_[pos] = static_cast<city_type>(state % n_);
		state = c.pred[state];
	}
	assert(state == 0 && "held_karp: backtrack must end at the start state");
	tour_[0] = city_type{0};
	std::fill_n(visited_.data(), n_, uint8_t{1});
	++tour_version_;

	// A dropped state may have completed it at a lower cost.
	cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = lost ? SolutionStatus::feasible : SolutionStatus::optimal;
	return *this;
}

template <DistanceSource Dist, typename Variant>
template <typename D>
auto Solver<Dist, Variant>::held_karp_labels(D& deadline) -> Solver& {
	// The table gives no tour until it is complete: back to the start city.
	auto give_up = [this]() -> Solver& {
		clear();
		append(city_type{0});
		return *this;
	};
	detail::Poller stop(deadline);
	if (stop()) return give_up();

	constexpr auto INF = std::numeric_limits<cost_type>::max();
	constexpr auto index_max = std::numeric_limits<std::uint32_t>::max();
	constexpr auto mask_bits = static_cast<std::size_t>(std::numeric_limits<std::size_t>::digits);

	n_ = dist_->size();
	assert(n_ <= mask_bits && "held_karp: instance too large (visited sets are std::size_t bitmasks)");
	ensure_capacity(n_);
	if constexpr (detail::dp_deadlines<Variant>::value)
		if (!start_dp_deadlines(stop)) return give_up();

	if (!hk_labels_cache_) hk_labels_cache_.emplace();
	auto& c = *hk_labels_cache_;
	c.masks.clear();
	c.index.clear();

	// Appends a slot for mask whose states hold no label yet.
	std::size_t state_cap = 0;
	auto new_slot = [&](std::size_t mask) -> std::uint32_t {
		const std::size_t slot = c.masks.size();
		const std::size_t end = (slot + 1) * n_;
		if (end > state_cap) {
			state_cap = std::max(end, 2 * state_cap);
			assert(state_cap <= index_max && "held_karp: too many states for 32-bit indices");
			if (c.first.size() < state_cap) {
				c.first.resize(state_cap);
				c.last.resize(state_cap);
			}
		}
		std::fill_n(c.first.begin() + static_cast<std::ptrdiff_t>(slot * n_), n_, 0);
		std::fill_n(c.last.begin() + static_cast<std::ptrdiff_t>(slot * n_), n_, 0);
		c.masks.push_back(mask);
		return static_cast<std::uint32_t>(slot);
	};

	// Appends a label, growing every per-label buffer, dimensions included,
	// geometrically.
	std::size_t labels = 0;
	std::size_t label_cap = 0;
	auto new_label = [&]() -> std::size_t {
		if (labels == label_cap) {
			label_cap = std::max(n_, 2 * label_cap);
			assert(label_cap <= index_max && "held_karp: too many labels for 32-bit indices");
			if (c.cost.size() < label_cap) {
				c.cost.resize(label_cap);
				c.pred.resize(label_cap);
				c.city.resize(label_cap);
			}
			ctx_.resize_dp(label_cap);
		}
		return labels++;
	};

	// Base: slot 0 is {0}, its one label at city 0
	new_slot(1);
	new_label();
	c.cost[0] = cost_type{};
	c.city[0] = 0;
	c.last[0] = 1;
	start_dp(0);

	const auto& v = variant();
	const std::size_t full = ~std::size_t{0} >> (mask_bits - n_);

	// Forward DP, one layer (set size) after another, as in held_karp_reachable.
	std::size_t layer_end = 1;
	for (std::size_t s = 0; s < c.masks.size(); ++s) {
		if (s == layer_end) {
			c.index.clear();
			layer_end = c.masks.size();
		}

		const std::size_t S = c.masks[s];

		// The cities of S that hold a label, gathered once for every target:
		// under a filter, most hold none.
		std::array<std::uint8_t, mask_bits> reached;
		std::size_t num_reached = 0;
		for (auto si = S; si; si &= si - 1) {
			const auto i = static_cast<std::size_t>(std::countr_zero(si));
			if (c.first[s * n_ + i] < c.last[s * n_ + i]) reached[num_reached++] = static_cast<std::uint8_t>(i);
		}

		// Every label of (S | {j}, j) comes from this slot: the target outside
		// keeps them contiguous, each compared only with those kept before it.
		for (auto sj = ~S & full; sj; sj &= sj - 1) {
			const auto j = static_cast<std::size_t>(std::countr_zero(sj));
			const auto cj = static_cast<city_type>(j);
			const std::size_t begin = labels;

			for (std::size_t k = 0; k < num_reached; ++k) {
				const std::size_t i = reached[k];
				const auto ci = static_cast<city_type>(i);
				const std::size_t from_state = s * n_ + i;

				for (std::size_t from = c.first[from_state]; from < c.last[from_state]; ++from) {
					if (c.cost[from] == INF) continue;
					if (stop()) return give_up();

					DPMove<city_type> move{ci, cj, S, from};
					ctx_.init(move, *dist_, c.cost[from]);
					invoke_prepare(v, move, ctx_);

					if (!invoke_filter(v, move, ctx_)) continue;

					const auto edge_cost = static_cast<cost_type>(static_cast<double>((*dist_)(ci, cj)) + ctx_.cost_delta);
					const cost_type new_cost = c.cost[from] + edge_cost;

					// Dropped if a kept label dominates it. Otherwise the labels it dominates
					// leave holes, and it takes the first hole of this target, if any.
					std::size_t into = labels;
					bool dominated = false;
					[[maybe_unused]] bool dropped_one = false;
					for (std::size_t k = begin; k < labels; ++k) {
						if (c.cost[k] == INF) {
							if (into == labels) into = k;
							continue;
						}
						const auto order = ctx_.compare_dp(k);
						if (c.cost[k] <= new_cost && order >= 0) {
							dominated = true;
							break;
						}
						if (new_cost <= c.cost[k] && order <= 0) {
							c.cost[k] = INF;
							dropped_one = true;
							if (into == labels) into = k;
						}
					}
					if (dominated) {
						// Dominance is transitive and kept labels dominate none of each other.
						assert(!dropped_one && "held_karp: a dominated label cannot dominate a kept one");
						continue;
					}
					// The labels it dominates leave the same cities open and depart no
					// earlier: dead with it, they are no loss.
					if (dp_dead(j, S | (std::size_t{1} << j))) continue;
					if (into == labels) into = new_label();

					c.cost[into] = new_cost;
					c.pred[into] = static_cast<std::uint32_t>(from);
					c.city[into] = static_cast<std::uint8_t>(j);
					move.to_state = into;
					ctx_.commit(move);
				}
			}

			// Slot of S | {j}, created by the first label kept for it.
			if (labels > begin) {
				const std::size_t next = S | (std::size_t{1} << j);
				const std::size_t t = c.index.find_or_insert(next, [&] { return new_slot(next); });
				c.first[t * n_ + j] = static_cast<std::uint32_t>(begin);
				c.last[t * n_ + j] = static_cast<std::uint32_t>(labels);
			}
		}
	}

	// Close the tour back to 0 from the full set, the last slot when reached.
	const std::size_t last = c.masks.size() - 1;
	cost_type best_cost = INF;
	std::size_t best_label = 0;

	if (c.masks[last] == full) {
		for (std::size_t i = 1; i < n_; ++i) {
			const std::size_t from_state = last * n_ + i;
			const auto ci = static_cast<city_type>(i);

			for (std::size_t from = c.first[from_state]; from < c.last[from_state]; ++from) {
				if (c.cost[from] == INF) continue;

				DPMove<city_type> move{ci, city_type{0}, full, from};
				ctx_.init(move, *dist_, c.cost[from]);
				invoke_prepare(v, move, ctx_);

				if (!invoke_filter(v, move, ctx_)) continue;

				const auto edge_cost = static_cast<cost_type>(static_cast<double>((*dist_)(ci, city_type{0})) + ctx_.cost_delta);
				const cost_type total = c.cost[from] + edge_cost;
				if (total < best_cost) {
					best_cost = total;
					best_label = from;
				}
			}
		}
	}

	if (best_cost == INF) {
		n_ = 0;
		cost_ = {};
		++tour_version_;
		status_ = SolutionStatus::infeasible;
		return *this;
	}

	// Backtrack through predecessor labels.
	std::size_t label = best_label;
	for (std::size_t pos = n_ - 1; pos > 0; --pos) {
		tour_[pos] = static_cast<city_type>(c.city[label]);
		label = c.pred[label];
	}
	assert(label == 0 && "held_karp: backtrack must end at the start label");
	tour_[0] = city_type{0};
	std::fill_n(visited_.data(), n_, uint8_t{1});
	++tour_version_;

	cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = SolutionStatus::optimal;
	return *this;
}

} // namespace periple
