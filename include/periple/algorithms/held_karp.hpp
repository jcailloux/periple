#pragma once

// Held-Karp algorithm (exact, dynamic programming)
//
// Bellman (1962), "Dynamic Programming Treatment of the Travelling Salesman Problem"
// Held, Karp (1962), "A Dynamic Programming Approach to Sequencing Problems"

#include <periple/core/dispatch.hpp>
#include <periple/core/solver.hpp>

#include <bit>
#include <limits>
#include <span>

namespace periple {
namespace detail {

template <DistanceSource Dist, typename Variant>
auto held_karp_solve(
	const Dist& dist, std::size_t n,
	std::span<typename dist_traits<Dist>::city_type> tour,
	std::span<typename dist_traits<Dist>::cost_type> dp,
	std::span<typename dist_traits<Dist>::city_type> parent,
	const Variant& variant)
	-> typename dist_traits<Dist>::cost_type
{
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;
	using move_type = DPMove<city_type, cost_type>;

	if (n <= 1) {
		if (n == 1) tour[0] = city_type{0};
		return cost_type{};
	}

	constexpr auto INF = std::numeric_limits<cost_type>::max();
	const std::size_t num_sets = std::size_t{1} << n;

	auto idx = [n](std::size_t S, std::size_t i) -> std::size_t {
		return S * n + i;
	};

	std::fill_n(dp.begin(), num_sets * n, INF);

	// Base: start at city 0
	dp[idx(1, 0)] = cost_type{};

	// Forward DP
	const std::size_t complement_mask = num_sets - 1;
	for (std::size_t S = 1; S < num_sets; S += 2) { // S += 2: city 0 always in set
		for (auto si = S; si; si &= si - 1) {
			auto i = static_cast<std::size_t>(std::countr_zero(si));
			if (dp[idx(S, i)] == INF) continue;

			auto ci = static_cast<city_type>(i);

			for (auto sj = ~S & complement_mask; sj; sj &= sj - 1) {
				auto j = static_cast<std::size_t>(std::countr_zero(sj));
				auto cj = static_cast<city_type>(j);
				auto raw_dist = dist(ci, cj);

				move_type move{ci, cj, dp[idx(S, i)], raw_dist, S};

				if (!dispatch_filter(variant, move)) continue;

				cost_type edge_cost = dispatch_eval<cost_type>(
					raw_dist, variant, move);

				std::size_t S_next = S | (std::size_t{1} << j);
				cost_type new_cost = dp[idx(S, i)] + edge_cost;
				if (new_cost < dp[idx(S_next, j)]) {
					dp[idx(S_next, j)]     = new_cost;
					parent[idx(S_next, j)] = ci;

					dispatch_on_move(variant, move);
				}
			}
		}
	}

	// Find optimal last city (close the tour back to 0)
	const std::size_t full = num_sets - 1;
	cost_type best_cost = INF;
	city_type best_last{};

	for (std::size_t i = 1; i < n; ++i) {
		if (dp[idx(full, i)] == INF) continue;

		auto ci = static_cast<city_type>(i);
		auto raw_dist = dist(ci, city_type{0});

		move_type move{ci, city_type{0}, dp[idx(full, i)], raw_dist, full};

		if (!dispatch_filter(variant, move)) continue;

		cost_type edge_cost = dispatch_eval<cost_type>(
			raw_dist, variant, move);

		cost_type c = dp[idx(full, i)] + edge_cost;
		if (c < best_cost) {
			best_cost = c;
			best_last = ci;
		}
	}

	// Backtrack
	if (best_cost < INF) {
		tour[n - 1] = best_last;
		std::size_t S = full;
		for (std::size_t pos = n - 1; pos > 0; --pos) {
			auto cur = static_cast<std::size_t>(tour[pos]);
			tour[pos - 1] = parent[idx(S, cur)];
			S ^= (std::size_t{1} << cur);
		}
	}

	return best_cost;
}

} // namespace detail

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::held_karp(HeldKarpParams)
	-> Solver&
{
	n_ = dist_->size();
	ensure_shared(n_);

	if (!hk_cache_) hk_cache_.emplace();

	const std::size_t num_sets = std::size_t{1} << n_;
	const std::size_t table_sz = num_sets * n_;
	hk_cache_->dp.resize(table_sz);
	hk_cache_->parent.resize(table_sz);

	auto best_cost = detail::held_karp_solve(
		*dist_, n_,
		std::span<city_type>(tour_.data(), n_),
		std::span<cost_type>(hk_cache_->dp.data(), table_sz),
		std::span<city_type>(hk_cache_->parent.data(), table_sz),
		variant_ref());

	if (best_cost == std::numeric_limits<cost_type>::max()) {
		n_ = 0;
		cost_ = {};
		status_ = SolutionStatus::infeasible;
		return *this;
	}

	cost_ = compute_tour_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = SolutionStatus::optimal;
	rebuild_position();
	return *this;
}

} // namespace periple
