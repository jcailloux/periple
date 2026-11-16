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
//   -- forward DP whose step s generates the paths of length s, section 1

#include <periple/core/solver.hpp>
#include <periple/core/moves/dp_move.hpp>

#include <algorithm>
#include <bit>
#include <cassert>
#include <concepts>
#include <cstdint>
#include <limits>
#include <span>
#include <type_traits>

namespace periple {

// held_karp engines. AllSets: a table over every visited set, up to 25 cities.
// ReachableSets: only the sets the variant's DPMove filter lets the DP reach,
// up to 64 cities; far faster when the filter prunes, slower when it prunes little.
struct AllSets {};
struct ReachableSets {};

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

// The engine of held_karp(): ReachableSets where a filter can prune, AllSets
// elsewhere, where ReachableSets would reach every set at a higher cost.
template <typename Variant, typename Ctx, typename CityT>
using default_hk_engine = std::conditional_t<filters_dp<Variant, Ctx, CityT>::value, ReachableSets, AllSets>;

} // namespace detail

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::held_karp(HeldKarpParams params) -> Solver& {
	return held_karp(detail::default_hk_engine<Variant, context_type, city_type>{}, params);
}

template <DistanceSource Dist, typename Variant>
template <typename Engine>
auto Solver<Dist, Variant>::held_karp(Engine, HeldKarpParams params) -> Solver& {
	static_assert(std::is_same_v<Engine, AllSets> || std::is_same_v<Engine, ReachableSets>,
		"held_karp: the engine is periple::AllSets or periple::ReachableSets");
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
		else return held_karp_reachable(deadline);
	});
}

// Stores at root the start city's state as append() leaves it, so a tour's DP
// cost and its replay by rebuild_and_cost start from the same state.
template <DistanceSource Dist, typename Variant>
void Solver<Dist, Variant>::start_dp(std::size_t root) {
	AppendMove<city_type> start{city_type{0}, city_type{}, 0};
	ctx_.init(start, *dist_, {tour_.data(), 0}, {position_.data(), 0}, cost_type{});
	invoke_prepare(variant(), start, ctx_);
	ctx_.begin_dp(root);
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
				if (new_cost < dp[idx(S_next, j)]) {
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

	cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = SolutionStatus::optimal;
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

				// Slot of S | {j}, created by the first accepted move into it.
				auto& t = c.target_slot[j];
				if (t == unresolved) {
					const std::size_t next = S | (std::size_t{1} << j);
					t = c.index.find_or_insert(next, [&] { return new_slot(next); });
				}

				const std::size_t to_state = std::size_t{t} * n_ + j;
				if (new_cost < c.cost[to_state]) {
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

	cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = SolutionStatus::optimal;
	return *this;
}

} // namespace periple
