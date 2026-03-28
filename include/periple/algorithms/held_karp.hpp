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

template <DistanceSource Dist, typename Variant>
auto Solver<Dist, Variant>::held_karp(HeldKarpParams) -> Solver& {
	if (try_trivial()) {
		status_ = SolutionStatus::optimal;
		return *this;
	}

	constexpr auto INF = std::numeric_limits<cost_type>::max();

	n_ = dist_->size();
	assert(n_ <= 25 && "held_karp: instance too large (exponential memory/time, max 25 cities)");
	ensure_capacity(n_);

	if (!hk_cache_) hk_cache_.emplace();

	const std::size_t num_sets = std::size_t{1} << n_;
	const std::size_t table_sz = num_sets * n_;
	hk_cache_->dp.resize(table_sz);
	hk_cache_->parent.resize(table_sz);

	auto dp     = std::span<cost_type>(hk_cache_->dp.data(), table_sz);
	auto parent = std::span<city_type>(hk_cache_->parent.data(), table_sz);

	auto idx = [this](std::size_t S, std::size_t i) -> std::size_t {
		return S * n_ + i;
	};

	std::fill_n(dp.begin(), table_sz, INF);

	// Base: start at city 0
	dp[idx(1, 0)] = cost_type{};

	const auto& variant = variant_ref();
	const std::span<const city_type> tour_span{tour_.data(), n_};
	const std::span<const city_type> pos_span{position_.data(), n_};

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
				auto raw_dist = (*dist_)(ci, cj);

				DPMove<city_type> move{ci, cj, S};
				ctx_.init(move, *dist_, tour_span, pos_span, dp[idx(S, i)]);
				invoke_prepare(variant, move, ctx_);

				if (!invoke_filter(variant, move, ctx_)) continue;

				cost_type edge_cost = static_cast<cost_type>(static_cast<double>(raw_dist) + ctx_.cost_delta);

				std::size_t S_next = S | (std::size_t{1} << j);
				cost_type new_cost = dp[idx(S, i)] + edge_cost;
				if (new_cost < dp[idx(S_next, j)]) {
					dp[idx(S_next, j)]     = new_cost;
					parent[idx(S_next, j)] = ci;
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

		DPMove<city_type> move{ci, city_type{0}, full};
		ctx_.init(move, *dist_, tour_span, pos_span, dp[idx(full, i)]);
		invoke_prepare(variant, move, ctx_);

		if (!invoke_filter(variant, move, ctx_)) continue;

		cost_type edge_cost = static_cast<cost_type>(static_cast<double>(raw_dist) + ctx_.cost_delta);

		cost_type c = dp[idx(full, i)] + edge_cost;
		if (c < best_cost) {
			best_cost = c;
			best_last = ci;
		}
	}

	// Backtrack
	if (best_cost < INF) {
		tour_[n_ - 1] = best_last;
		std::size_t S = full;
		for (std::size_t pos = n_ - 1; pos > 0; --pos) {
			auto cur = static_cast<std::size_t>(tour_[pos]);
			tour_[pos - 1] = parent[idx(S, cur)];
			S ^= (std::size_t{1} << cur);
		}
	}

	if (best_cost == INF) {
		n_ = 0;
		cost_ = {};
		status_ = SolutionStatus::infeasible;
		return *this;
	}

	cost_ = rebuild_and_cost(std::span<const city_type>(tour_.data(), n_));
	status_ = SolutionStatus::optimal;
	return *this;
}

} // namespace periple