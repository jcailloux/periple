#pragma once

// Held-Karp algorithm (exact, dynamic programming)
//
// Bellman (1962), "Dynamic Programming Treatment of the Travelling Salesman Problem"
// Held, Karp (1962), "A Dynamic Programming Approach to Sequencing Problems"

#include <periple/core/solver.hpp>

#include <bit>
#include <limits>
#include <span>

namespace periple {
namespace detail {

template <DistanceSource Dist>
auto held_karp_solve(
	const Dist& dist, std::size_t n,
	std::span<typename dist_traits<Dist>::city_type> tour,
	std::span<typename dist_traits<Dist>::cost_type> dp,
	std::span<typename dist_traits<Dist>::city_type> parent)
	-> typename dist_traits<Dist>::cost_type
{
	using cost_type = typename dist_traits<Dist>::cost_type;
	using city_type = typename dist_traits<Dist>::city_type;

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
		// Iterate only over cities in S
		for (auto si = S; si; si &= si - 1) {
			auto i = static_cast<std::size_t>(std::countr_zero(si));
			if (dp[idx(S, i)] == INF) continue; // unreachable state

			// Iterate only over cities not in S
			for (auto sj = ~S & complement_mask; sj; sj &= sj - 1) {
				auto j = static_cast<std::size_t>(std::countr_zero(sj));
				std::size_t S_next = S | (std::size_t{1} << j);
				cost_type new_cost = dp[idx(S, i)] +
					dist(static_cast<city_type>(i),
						 static_cast<city_type>(j));
				if (new_cost < dp[idx(S_next, j)]) {
					dp[idx(S_next, j)]     = new_cost;
					parent[idx(S_next, j)] = static_cast<city_type>(i);
				}
			}
		}
	}

	// Find optimal last city
	const std::size_t full = num_sets - 1;
	cost_type best_cost = INF;
	city_type best_last{};

	for (std::size_t i = 1; i < n; ++i) {
		cost_type c = dp[idx(full, i)] +
			dist(static_cast<city_type>(i), city_type{0});
		if (c < best_cost) {
			best_cost = c;
			best_last = static_cast<city_type>(i);
		}
	}

	// Backtrack
	tour[n - 1] = best_last;
	std::size_t S = full;
	for (std::size_t pos = n - 1; pos > 0; --pos) {
		auto cur = static_cast<std::size_t>(tour[pos]);
		tour[pos - 1] = parent[idx(S, cur)];
		S ^= (std::size_t{1} << cur);
	}

	return best_cost;
}

} // namespace detail

template <DistanceSource Dist, typename TourCost>
inline auto Solver<Dist, TourCost>::held_karp(HeldKarpParams) -> Solver&
{
	n_ = dist_->size();
	ensure_shared(n_);

	if (!hk_cache_) hk_cache_.emplace();

	const std::size_t num_sets = std::size_t{1} << n_;
	const std::size_t table_sz = num_sets * n_;
	hk_cache_->dp.resize(table_sz);
	hk_cache_->parent.resize(table_sz);

	detail::held_karp_solve(
		*dist_, n_,
		std::span<city_type>(tour_.data(), n_),
		std::span<cost_type>(hk_cache_->dp.data(), table_sz),
		std::span<city_type>(hk_cache_->parent.data(), table_sz));

	cost_ = compute_tour_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = SolutionStatus::optimal;
	rebuild_position();
	return *this;
}

} // namespace periple
